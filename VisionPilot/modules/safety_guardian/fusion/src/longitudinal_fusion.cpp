#include <fusion/longitudinal_fusion.hpp>
#include <logging/logger.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace visionpilot::fusion {

// ─── Construction ─────────────────────────────────────────────────────────────

LongitudinalFusion::LongitudinalFusion()
    : LongitudinalFusion(Config{})
{}

LongitudinalFusion::LongitudinalFusion(Config cfg)
    : cfg_(cfg)
    , rng_(std::random_device{}())
{
    if (cfg_.n_particles < 10)
        throw std::invalid_argument("LongitudinalFusion: n_particles must be >= 10");
    particles_.reserve(static_cast<std::size_t>(cfg_.n_particles));
}

// ─── Public API ───────────────────────────────────────────────────────────────

void LongitudinalFusion::reset()
{
    particles_.clear();
    initialised_ = false;
    hits_ = 0;
    misses_ = 0;
    tracked_ = false;
}

CIPOFusionEstimate LongitudinalFusion::update(
    const models::AutoDriveOutput& autodrive,
    const models::AutoSpeedOutput& autospeed,
    const cv::Mat& /*preprocessed_frame*/,
    float dt_s)
{
    // ── Step 2: AutoSpeed → world distance via homography ────────────────────
    CIPOFusionEstimate est;
    Meas cipo_raw;
    if (autospeed.valid) {
        const auto sel = select_cipo(autospeed.detections);
        cipo_raw            = sel.meas;
        est.cut_in_detected = sel.cut_in;
        if (cipo_raw.valid) {
            est.cipo_raw_found  = true;
            est.cipo_raw_dist_m = cipo_raw.distance_m;
        }
    }

    // Temporal tracking state: confirmation + coasting
    // A target is confirmed after 2 consecutive detection hits.
    // Once tracked, it coasts across brief detection dropouts (up to 6 frames ~0.2s)
    // without resetting or jumping to max distance.
    if (cipo_raw.valid) {
        hits_++;
        misses_ = 0;
        if (hits_ >= 2) {
            tracked_ = true;
        }
    } else {
        misses_++;
        hits_ = std::max(0, hits_ - 1);
        if (misses_ >= 6) {
            tracked_ = false;
        }
    }

    static constexpr float D_MAX = 150.f;

    // If no target is tracked: report clear highway and reset filter
    if (!tracked_) {
        if (initialised_) {
            VP_INFO("[Fusion] Target lost (misses=%d) — reset to %.0f m", misses_, D_MAX);
            reset();
        }
        est.valid       = false;
        est.distance_m  = D_MAX;
        est.velocity_ms = 0.0f;
        return est;
    }

    // ── Step 4: Particle filter ───────────────────────────────────────────────
    const float dt = (dt_s > 1e-6f) ? dt_s : cfg_.dt_s;

    if (!initialised_) {
        init_from(cipo_raw.distance_m, cfg_.cipo_noise_m);
        initialised_ = true;
    } else {
        predict(dt);

        if (cipo_raw.valid) {
            float cloud_mean = 0.f;
            for (const auto& p : particles_) cloud_mean += p.distance_m;
            cloud_mean /= static_cast<float>(particles_.size());

            // Re-init only on sudden true target jump (e.g. cut-in car ahead by > 25m)
            if (std::abs(cipo_raw.distance_m - cloud_mean) > cfg_.reset_gate_m) {
                VP_INFO("[Fusion] Target change — reinit %.1f→%.1f m (CIPO)  gate=%.1f",
                        cloud_mean, cipo_raw.distance_m, cfg_.reset_gate_m);
                init_from(cipo_raw.distance_m, cfg_.cipo_noise_m);
            }
        }
    }

    if (cipo_raw.valid) {
        weight_update(Meas{}, cipo_raw);
        if (effective_n() < 0.5f * static_cast<float>(cfg_.n_particles)) resample();
    }

    // ── Step 5: Posterior mean — weighted particle average (MRPT getMean style) ─
    // Both distance and velocity come directly from the particle ensemble.
    // No EMA, no finite difference, no deadband needed.
    const auto w = linear_weights();
    const auto N = particles_.size();
    float mean_d = 0.f, mean_v = 0.f;
    for (std::size_t i = 0; i < N; ++i) {
        mean_d += w[i] * particles_[i].distance_m;
        mean_v += w[i] * particles_[i].velocity_ms;
    }
    float var_d = 0.f;
    for (std::size_t i = 0; i < N; ++i) {
        const float dd = particles_[i].distance_m - mean_d;
        var_d += w[i] * dd * dd;
    }

    est.valid             = true;
    est.distance_m        = mean_d;
    est.velocity_ms       = mean_v;
    est.distance_stddev_m = std::sqrt(std::max(0.f, var_d));

    // ── Step 7: Debug log ─────────────────────────────────────────────────────
    if (cfg_.debug) {
        char cr_buf[32];
        if (cipo_raw.valid)
            std::snprintf(cr_buf, sizeof(cr_buf), "%.1f m", cipo_raw.distance_m);
        else
            std::snprintf(cr_buf, sizeof(cr_buf), "(none)");

        VP_INFO("[Fusion] AD_prob=%.0f%% | CIPO=%s%s (hits=%d miss=%d) | Fused=%.1f m  v=%.2f m/s  ±%.1f m",
                autodrive.valid ? autodrive.flag_prob * 100.f : 0.f,
                cr_buf,
                est.cut_in_detected ? " [CUT-IN]" : "",
                hits_, misses_,
                est.distance_m, est.velocity_ms, est.distance_stddev_m);
    }

    return est;
}

// ─── Homography projection ────────────────────────────────────────────────────

float LongitudinalFusion::project_dist(const cv::Mat& H, float ux, float uy)
{
    std::vector<cv::Point2f> src = {cv::Point2f(ux, uy)}, dst;
    cv::perspectiveTransform(src, dst, H);
    return std::sqrt(dst[0].x * dst[0].x + dst[0].y * dst[0].y);
}

// Find the closest Level 1 and closest Level 2 detection.
// If Level 2 is closer than Level 1 → cut-in: fuse the Level 2 distance.
// Otherwise fuse the closest Level 1.
// class_id == 1: CIPO Level 1 (straight ahead, same lane)
// class_id == 2: CIPO Level 2 (adjacent lane, potential cut-in)
LongitudinalFusion::CIPOSelection
LongitudinalFusion::select_cipo(const std::vector<models::Detection>& dets) const
{
    float best_l1 = std::numeric_limits<float>::max();
    float best_l2 = std::numeric_limits<float>::max();

    for (const auto& d : dets) {
        if (d.class_id != 1 && d.class_id != 2) continue;

        // Bounding box geometry sanity checks
        const float bw = d.x2 - d.x1;
        const float bh = d.y2 - d.y1;
        if (bw < 15.f || bh < 12.f) continue;
        const float aspect = bw / bh;
        if (aspect < 0.4f || aspect > 3.2f) continue;

        const float cx = (d.x1 + d.x2) * 0.5f;

        // Project bottom-center point to world coordinates: (x_forward [m], y_lateral [m, +left])
        std::vector<cv::Point2f> src = {cv::Point2f(cx, d.y2)}, dst;
        cv::perspectiveTransform(src, dst, H_);
        const float x_fwd = dst[0].x;
        const float y_lat = dst[0].y;

        // Object must be ahead of ego vehicle and within trackable horizon (max 95m)
        if (x_fwd < 2.0f || x_fwd > 95.0f) continue;

        // Distance-scaled confidence gate:
        // Far field (> 75m) requires higher confidence (>= 0.66) to reject distant horizon noise.
        // Mid/near field (<= 75m) accepts standard detection confidence (>= 0.55).
        if (x_fwd > 75.0f && d.score < 0.66f) continue;
        if (d.score < 0.55f) continue;

        // Physical vehicle width validation via H:
        // Project bottom corners to get estimated physical width on ground
        std::vector<cv::Point2f> corners_src = {cv::Point2f(d.x1, d.y2), cv::Point2f(d.x2, d.y2)}, corners_dst;
        cv::perspectiveTransform(corners_src, corners_dst, H_);
        const float phys_width = std::abs(corners_dst[1].y - corners_dst[0].y);
        // Plausible motor vehicle width: 0.7m (motorcycle) to 3.6m (bus/truck)
        // Discard flat road textures / grass tufts (< 0.7m) or multi-lane artifacts (> 3.6m)
        if (phys_width < 0.7f || phys_width > 3.6f) {
            if (cfg_.debug) {
                VP_INFO("[CIPO-DBG] Rejecting unphysical width: cls=%d score=%.2f w_phys=%.2fm",
                        d.class_id, d.score, phys_width);
            }
            continue;
        }

        // Lane corridor gating:
        // Class 1 (in-lane lead): must be within ego lane corridor (|y_lateral| <= 2.0m)
        // Class 2 (cut-in): must be in adjacent lane corridor (1.0m < |y_lateral| <= 4.5m)
        if (d.class_id == 1 && std::abs(y_lat) > 2.0f) continue;
        if (d.class_id == 2 && (std::abs(y_lat) < 1.0f || std::abs(y_lat) > 4.5f)) continue;

        const float dist = std::sqrt(x_fwd * x_fwd + y_lat * y_lat);
        if (cfg_.debug) {
            VP_INFO("[CIPO-DBG] cls=%d score=%.2f bbox=(%.0f,%.0f,%.0f,%.0f) → fwd=%.1fm lat=%+.1fm dist=%.1fm",
                    d.class_id, d.score, d.x1, d.y1, d.x2, d.y2, x_fwd, y_lat, dist);
        }
        if (d.class_id == 1 && dist < best_l1) best_l1 = dist;
        if (d.class_id == 2 && dist < best_l2) best_l2 = dist;
    }

    CIPOSelection sel;
    sel.meas.stddev_m = cfg_.cipo_noise_m;

    const bool have_l1 = best_l1 < std::numeric_limits<float>::max();
    const bool have_l2 = best_l2 < std::numeric_limits<float>::max();

    if (have_l2 && have_l1 && best_l2 < best_l1) {
        sel.meas.distance_m = best_l2;
        sel.meas.valid      = true;
        sel.cut_in          = true;
    } else if (have_l1) {
        sel.meas.distance_m = best_l1;
        sel.meas.valid      = true;
    }
    return sel;
}

// ─── Particle filter internals ────────────────────────────────────────────────

void LongitudinalFusion::init_from(float dist_m, float stddev_m)
{
    particles_.resize(static_cast<std::size_t>(cfg_.n_particles));
    std::normal_distribution<float> nd(dist_m, stddev_m);
    std::normal_distribution<float> nv(0.f, 2.f);
    for (auto& p : particles_) {
        p.distance_m  = std::clamp(nd(rng_), 0.f, cfg_.d_max_m);
        p.velocity_ms = nv(rng_);
        p.log_w       = 0.f;
    }
}

void LongitudinalFusion::predict(float dt_s)
{
    std::normal_distribution<float> nd(0.f, cfg_.process_noise_dist_m);
    std::normal_distribution<float> nv(0.f, cfg_.process_noise_vel_ms);
    for (auto& p : particles_) {
        p.distance_m  = std::clamp(p.distance_m + p.velocity_ms * dt_s + nd(rng_), 0.f, cfg_.d_max_m);
        p.velocity_ms = p.velocity_ms + nv(rng_);
    }
}

float LongitudinalFusion::gaussian_loglik(float z, float mean, float sigma)
{
    const float d = z - mean;
    return -0.5f * (d / sigma) * (d / sigma);
}

void LongitudinalFusion::weight_update(const Meas& ad, const Meas& cipo_raw)
{
    for (auto& p : particles_) {
        if (cipo_raw.valid) {
            p.log_w += gaussian_loglik(cipo_raw.distance_m, p.distance_m, cipo_raw.stddev_m);
        } else if (ad.valid) {
            p.log_w += gaussian_loglik(ad.distance_m, p.distance_m, ad.stddev_m);
        }
    }
}

std::vector<float> LongitudinalFusion::linear_weights() const
{
    const std::size_t N = particles_.size();
    float max_lw = particles_[0].log_w;
    for (const auto& p : particles_) max_lw = std::max(max_lw, p.log_w);

    std::vector<float> w(N);
    float sum = 0.f;
    for (std::size_t i = 0; i < N; ++i) {
        w[i] = std::exp(particles_[i].log_w - max_lw);
        sum  += w[i];
    }
    if (sum < 1e-12f) {
        const float w0 = 1.f / static_cast<float>(N);
        for (auto& wi : w) wi = w0;
    } else {
        for (auto& wi : w) wi /= sum;
    }
    return w;
}

float LongitudinalFusion::effective_n() const
{
    const auto w = linear_weights();
    float ss = 0.f;
    for (auto wi : w) ss += wi * wi;
    return 1.f / (ss + 1e-12f);
}

void LongitudinalFusion::resample()
{
    const int N = static_cast<int>(particles_.size());
    if (N == 0) return;

    const auto w = linear_weights();
    std::vector<float> cs(static_cast<std::size_t>(N));
    cs[0] = w[0];
    for (int i = 1; i < N; ++i)
        cs[static_cast<std::size_t>(i)] =
            cs[static_cast<std::size_t>(i-1)] + w[static_cast<std::size_t>(i)];

    std::vector<Particle> np;
    np.reserve(static_cast<std::size_t>(N));
    std::uniform_real_distribution<float> u(0.f, 1.f / static_cast<float>(N));
    const float u0 = u(rng_);
    int j = 0;
    for (int i = 0; i < N; ++i) {
        const float thr = u0 + static_cast<float>(i) / static_cast<float>(N);
        while (j < N-1 && cs[static_cast<std::size_t>(j)] < thr) ++j;
        np.push_back({particles_[static_cast<std::size_t>(j)].distance_m,
                      particles_[static_cast<std::size_t>(j)].velocity_ms,
                      0.f});
    }
    particles_ = std::move(np);
}

}  // namespace visionpilot::fusion
