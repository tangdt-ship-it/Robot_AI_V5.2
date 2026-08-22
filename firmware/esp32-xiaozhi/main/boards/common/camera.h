#ifndef CAMERA_H
#define CAMERA_H

#include <string>

struct CameraNavigationMetrics {
    bool valid = false;
    
    // Gradient/Luma based traversability (existing)
    float left_open_pct = 0.0f;
    float center_open_pct = 0.0f;
    float right_open_pct = 0.0f;
    float corridor_offset = 0.0f;  // -1.0 (blocked left) to +1.0 (blocked right)
    float confidence_pct = 0.0f;
    unsigned sampled_pixels = 0;
    
    // Optical Flow & Collision Detection (NEW)
    float time_to_collision_ms = 0.0f;  // Estimated ms until collision at current speed
    float motion_flow_magnitude = 0.0f;  // Optical flow magnitude (0-100)
    bool collision_imminent = false;     // true if ttc < 500ms or flow > 80
    
    // Edge Detection & Obstacle Classification (NEW)
    float edge_density_pct = 0.0f;      // Edge map density (high = obstacles)
    float obstacle_center_x = 0.0f;     // Obstacle X position (0=left, 1=right) or -1=none
    float obstacle_height_pct = 0.0f;   // Vertical extent of obstacle (0-100)
    
    // HSV Color Segmentation (NEW) - obstacle type detection
    bool dark_obstacle = false;         // High density of dark pixels (shadow/black object)
    bool high_contrast = false;         // High contrast edge (sharp boundary)
    float obstacle_left_pct = 0.0f;     // % likelihood obstacle on left side
    float obstacle_right_pct = 0.0f;    // % likelihood obstacle on right side
};

class Camera {
public:
    virtual void SetExplainUrl(const std::string& url, const std::string& token) = 0;
    virtual bool Capture() = 0;
    virtual bool SetHMirror(bool enabled) = 0;
    virtual bool SetVFlip(bool enabled) = 0;
    virtual bool SetSwapBytes(bool enabled) { return false; }  // Optional, default no-op
    virtual bool AnalyzeNavigation(CameraNavigationMetrics& metrics) {
        metrics = {};
        return false;
    }
    // Acquires one transaction spanning Capture() and Explain().  This is
    // deliberately optional for non-ESP cameras; callers must fail safe when
    // the production camera cannot provide serialization.
    virtual bool TryAcquireVision(uint32_t timeout_ms) { (void)timeout_ms; return false; }
    virtual void ReleaseVision() {}
    virtual std::string Explain(const std::string& question) = 0;
};

#endif // CAMERA_H
