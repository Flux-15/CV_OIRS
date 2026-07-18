"""
OIRS 4-Mirror Auto-Alignment -- HUD & Debug Overlay
=====================================================
Renders maximum visual information on the OpenCV video frame:
  - 3D-to-2D projections of static Node 1 (TX) and dynamic Node 2 (RX)
  - Optical beam paths, reflection normals, and bisection angle math
  - Real-time 4-mirror telemetry table with error & lock status
  - 2x2 graphical mirror array visualizer (dial/bar gauges)
  - Mode banners, system FPS, and geometry readouts
"""

import cv2
import numpy as np
import config

# Color Palette (BGR)
COLOR_BG_DARK = (20, 20, 20)
COLOR_TEXT_WHITE = (245, 245, 245)
COLOR_TEXT_MUTED = (160, 160, 160)
COLOR_TX_GREEN = (0, 230, 0)       # Node 1 (TX) -- Bright Green
COLOR_RX_ORANGE = (0, 140, 255)    # Node 2 (RX) -- Vibrant Orange
COLOR_OPTICAL_CYAN = (255, 255, 0) # Optical path / Bisector -- Cyan
COLOR_LOCKED = (0, 220, 0)         # Locked status -- Green
COLOR_TRACKING = (0, 220, 255)     # Tracking status -- Yellow/Amber
COLOR_LOST = (50, 50, 250)         # Lost / Alert -- Red
COLOR_GRID_LINE = (70, 70, 70)


def _draw_translucent_box(frame, x1, y1, x2, y2, color=COLOR_BG_DARK, alpha=0.68):
    """Draw a rounded semi-transparent rectangle on the frame."""
    h, w = frame.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    if x1 >= x2 or y1 >= y2:
        return
    roi = frame[y1:y2, x1:x2]
    overlay = np.full_like(roi, color)
    cv2.addWeighted(overlay, alpha, roi, 1.0 - alpha, 0, roi)
    cv2.rectangle(frame, (x1, y1), (x2, y2), (100, 100, 100), 1, cv2.LINE_AA)


def _project_world_to_cam(world_pos, camera_matrix, dist_coeffs):
    """Project a 3D point in World Frame {W} to 2D image pixel (u, v)."""
    if camera_matrix is None or dist_coeffs is None:
        return None
    try:
        # P_cam = R_w2c @ (world_pos - camera_pos_world)
        R_w2c = config.R_CAM_TO_WORLD.T
        rvec, _ = cv2.Rodrigues(R_w2c)
        tvec = -R_w2c @ config.CAMERA_POSITION_WORLD
        
        pts_3d = np.array([world_pos], dtype=np.float64)
        img_pts, _ = cv2.projectPoints(pts_3d, rvec, tvec, camera_matrix, dist_coeffs)
        u, v = int(round(img_pts[0, 0, 0])), int(round(img_pts[0, 0, 1]))
        return (u, v)
    except Exception:
        return None


def _draw_mirror_grid_visualizer(frame, x, y, w, h, controller, target_pans, target_tilts):
    """Draw a 2x2 HUD graphical widget representing the 4 mirrors in real time."""
    _draw_translucent_box(frame, x, y, x + w, y + h, alpha=0.75)
    
    cv2.putText(frame, "2x2 MIRROR ARRAY VISUALIZER", (x + 12, y + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, COLOR_TEXT_WHITE, 1, cv2.LINE_AA)
    cv2.line(frame, (x, y + 30), (x + w, y + 30), COLOR_GRID_LINE, 1)
    
    grid_w = (w - 30) // 2
    grid_h = (h - 45) // 2
    
    # Mirror positions in widget: Top-Left (M0), Top-Right (M1), Bottom-Left (M2), Bottom-Right (M3)
    offsets = [
        (x + 10, y + 38),              # M0
        (x + 20 + grid_w, y + 38),     # M1
        (x + 10, y + 44 + grid_h),     # M2
        (x + 20 + grid_w, y + 44 + grid_h) # M3
    ]
    
    for i in range(config.NUM_MIRRORS):
        mx, my = offsets[i]
        sp = controller.pans[i]
        st = controller.tilts[i]
        tp = target_pans[i] if target_pans else sp
        tt = target_tilts[i] if target_tilts else st
        
        err = max(abs(tp - sp), abs(tt - st))
        status_color = COLOR_LOCKED if err < 0.5 else COLOR_TRACKING
        if not target_pans:
            status_color = COLOR_TEXT_MUTED
            
        # Box for mirror i
        cv2.rectangle(frame, (mx, my), (mx + grid_w, my + grid_h), status_color, 1, cv2.LINE_AA)
        
        # Header label M0..M3
        cv2.putText(frame, f"M{i}", (mx + 6, my + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2, cv2.LINE_AA)
        
        # Pan / Tilt values
        cv2.putText(frame, f"P:{sp:5.1f} (T:{tp:5.1f})", (mx + 30, my + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, COLOR_TEXT_WHITE, 1)
        cv2.putText(frame, f"T:{st:5.1f} (T:{tt:5.1f})", (mx + 30, my + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.38, COLOR_TEXT_WHITE, 1)
        
        # Small dial gauge for pan angle deflection from 90 deg
        center_x, center_y = mx + grid_w - 25, my + grid_h - 18
        cv2.circle(frame, (center_x, center_y), 10, (80, 80, 80), 1)
        angle_rad = np.radians(sp - 90.0)
        dx = int(8 * np.sin(angle_rad))
        dy = int(-8 * np.cos(angle_rad))
        cv2.line(frame, (center_x, center_y), (center_x + dx, center_y + dy), status_color, 2, cv2.LINE_AA)


def draw_debug_overlay(frame, state, control_loop, controller, camera_matrix=None, dist_coeffs=None):
    """
    Master drawing function: overlays all 3D telemetry, optical paths, and HUD tables.
    """
    h, w = frame.shape[:2]
    
    with state["lock"]:
        all_det = list(state.get("all_detections", []))
        mode = state.get("mode", "NO_NODE2")
        node2_pos = state.get("node2_pos")
        target_pans = state.get("target_pans")
        target_tilts = state.get("target_tilts")
        all_n_des = state.get("all_n_des")
        node1_pos = state.get("node1_pos", config.NODE1_POSITION_WORLD)


    # ─── 1. OPTICAL PATHS & 3D PROJECTIONS IN CAMERA VIEW ────────────────────────
    # A. Draw or Project Node 1 (TX)
    tx_pixel = None
    node1_det_obj = next((d for d in all_det if d.marker_id == config.MARKER_ID_NODE_1), None)
    if node1_det_obj is not None:
        corners_int = node1_det_obj.corners.astype(int)
        cv2.polylines(frame, [corners_int], True, COLOR_TX_GREEN, 2, cv2.LINE_AA)
        tx_pixel = (int(node1_det_obj.centroid[0]), int(node1_det_obj.centroid[1]))
        cv2.drawMarker(frame, tx_pixel, COLOR_TX_GREEN, cv2.MARKER_DIAMOND, 18, 2, cv2.LINE_AA)
        cv2.circle(frame, tx_pixel, 14, COLOR_TX_GREEN, 2, cv2.LINE_AA)
        cv2.putText(frame, "[TX] NODE 1 (ARUCO DETECTED)", (tx_pixel[0] + 18, tx_pixel[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_TX_GREEN, 2, cv2.LINE_AA)
        dist_tx = np.linalg.norm(node1_pos)
        cv2.putText(frame, f"Dist: {dist_tx:.2f}m | Pos: [{node1_pos[0]:.2f}, {node1_pos[1]:.2f}, {node1_pos[2]:.2f}]", (tx_pixel[0] + 18, tx_pixel[1] + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT_WHITE, 1, cv2.LINE_AA)
    else:
        tx_pixel = _project_world_to_cam(node1_pos, camera_matrix, dist_coeffs)
        if tx_pixel and 0 <= tx_pixel[0] <= w and 0 <= tx_pixel[1] <= h:
            cv2.drawMarker(frame, tx_pixel, COLOR_TX_GREEN, cv2.MARKER_DIAMOND, 18, 2, cv2.LINE_AA)
            cv2.circle(frame, tx_pixel, 12, COLOR_TX_GREEN, 1, cv2.LINE_AA)
            cv2.putText(frame, "[TX] NODE 1 (STORED/DEFAULT)", (tx_pixel[0] + 15, tx_pixel[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_TX_GREEN, 2, cv2.LINE_AA)
            cv2.putText(frame, f"Pos: [{node1_pos[0]:.2f}, {node1_pos[1]:.2f}, {node1_pos[2]:.2f}]m", (tx_pixel[0] + 15, tx_pixel[1] + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT_WHITE, 1, cv2.LINE_AA)

    # B. Draw ArUco Detections & Node 2 (RX)
    rx_pixel = None
    for d in all_det:
        if d.marker_id == config.MARKER_ID_NODE_1:
            continue  # Already drawn above
        color = COLOR_RX_ORANGE if d.marker_id == config.MARKER_ID_NODE_2 else COLOR_TEXT_MUTED
        corners_int = d.corners.astype(int)
        cv2.polylines(frame, [corners_int], True, color, 2, cv2.LINE_AA)
        cu, cv_ = int(d.centroid[0]), int(d.centroid[1])
        
        if d.marker_id == config.MARKER_ID_NODE_2:
            rx_pixel = (cu, cv_)
            cv2.drawMarker(frame, rx_pixel, color, cv2.MARKER_CROSS, 16, 2, cv2.LINE_AA)
            cv2.circle(frame, rx_pixel, 18, color, 2, cv2.LINE_AA)
            cv2.putText(frame, "[RX] NODE 2 (DYNAMIC)", (cu + 22, cv_ - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
            if d.position_world is not None:
                dist = np.linalg.norm(d.position_world)
                az = np.degrees(np.arctan2(d.position_world[0], d.position_world[1]))
                el = np.degrees(np.arctan2(d.position_world[2], d.position_world[1]))
                cv2.putText(frame, f"Dist: {dist:.2f}m | Az: {az:+.1f} | El: {el:+.1f}", (cu + 22, cv_ + 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_TEXT_WHITE, 1, cv2.LINE_AA)
        else:
            cv2.putText(frame, f"ID:{d.marker_id}", (cu + 10, cv_ - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    # C. Draw Bisector Optical Beam Line (if both TX and RX visible on screen)
    if tx_pixel and rx_pixel and 0 <= tx_pixel[0] <= w and 0 <= tx_pixel[1] <= h:
        cv2.line(frame, tx_pixel, rx_pixel, COLOR_OPTICAL_CYAN, 2, cv2.LINE_AA)
        mid_pt = ((tx_pixel[0] + rx_pixel[0]) // 2, (tx_pixel[1] + rx_pixel[1]) // 2)
        cv2.circle(frame, mid_pt, 6, COLOR_OPTICAL_CYAN, -1, cv2.LINE_AA)
        
        # Compute optical angles (incidence & reflection)
        if node2_pos is not None:
            s_vec = node1_pos - np.zeros(3) # vector from array center to TX
            r_vec = node2_pos - np.zeros(3)                   # vector from array center to RX
            s_hat = s_vec / np.linalg.norm(s_vec)
            r_hat = r_vec / np.linalg.norm(r_vec)
            n_avg = (s_hat + r_hat) / np.linalg.norm(s_hat + r_hat)
            inc_deg = np.degrees(np.arccos(np.clip(np.dot(s_hat, n_avg), -1.0, 1.0)))
            
            label = f"Bisector Normal | Inc=Ref: {inc_deg:.1f} deg"
            cv2.putText(frame, label, (mid_pt[0] - 80, mid_pt[1] - 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_OPTICAL_CYAN, 2, cv2.LINE_AA)

    # ─── 2. TOP DASHBOARD BANNER ──────────────────────────────────────────────
    _draw_translucent_box(frame, 0, 0, w, 40, alpha=0.82)
    cv2.putText(frame, "OIRS 4-MIRROR AUTO-ALIGNMENT SYSTEM [IK MODE]", (15, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, COLOR_TEXT_WHITE, 2, cv2.LINE_AA)
                
    ctrl_status = control_loop.get_status()
    badge_color = COLOR_LOCKED if "LOCKED" in ctrl_status else (COLOR_TRACKING if "TRACKING" in ctrl_status else COLOR_LOST)
    badge_text = ctrl_status if ctrl_status else mode
    
    # Right-aligned status badge
    text_size = cv2.getTextSize(badge_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
    bx1 = w - text_size[0] - 30
    cv2.rectangle(frame, (bx1, 7), (w - 10, 33), badge_color, -1, cv2.LINE_AA)
    cv2.putText(frame, badge_text, (bx1 + 10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)

    # ─── 3. LEFT HUD PANEL: TELEMETRY & IK TABLE ──────────────────────────────
    panel_w = 410
    panel_h = 245
    px, py = 15, 55
    _draw_translucent_box(frame, px, py, px + panel_w, py + panel_h, alpha=0.75)
    
    cv2.putText(frame, "REAL-TIME 4-MIRROR TELEMETRY", (px + 12, py + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_TEXT_WHITE, 2, cv2.LINE_AA)
    cv2.line(frame, (px, py + 32), (px + panel_w, py + 32), COLOR_GRID_LINE, 1)
    
    # Optical geometry summary
    y_off = py + 52
    if node2_pos is not None:
        dist_tx_rx = np.linalg.norm(node2_pos - node1_pos)
        cv2.putText(frame, f"TX->RX Path Length: {dist_tx_rx:.2f} m", (px + 12, y_off),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_OPTICAL_CYAN, 1, cv2.LINE_AA)
    else:
        cv2.putText(frame, "RX Node 2 Not Detected -- Holding Angles", (px + 12, y_off),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_LOST, 1, cv2.LINE_AA)
    
    y_off += 25
    # Table Header
    cv2.putText(frame, "MIR | TARGET (Pan,Tilt) | ACTUAL (Pan,Tilt) | ERR | STAT", (px + 12, y_off),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, COLOR_TEXT_MUTED, 1, cv2.LINE_AA)
    cv2.line(frame, (px + 10, y_off + 6), (px + panel_w - 10, y_off + 6), COLOR_GRID_LINE, 1)
    
    # Table Rows
    for i in range(config.NUM_MIRRORS):
        y_off += 24
        tp = target_pans[i] if target_pans else controller.pans[i]
        tt = target_tilts[i] if target_tilts else controller.tilts[i]
        sp = controller.pans[i]
        st = controller.tilts[i]
        err = max(abs(tp - sp), abs(tt - st))
        stat_str = "LCK" if err < 0.5 else "TRK"
        stat_col = COLOR_LOCKED if err < 0.5 else COLOR_TRACKING
        
        row_str = f" M{i} |   {tp:5.1f} , {tt:5.1f}   |   {sp:5.1f} , {st:5.1f}   | {err:3.1f} |"
        cv2.putText(frame, row_str, (px + 12, y_off), cv2.FONT_HERSHEY_SIMPLEX, 0.42, COLOR_TEXT_WHITE, 1, cv2.LINE_AA)
        cv2.putText(frame, stat_str, (px + 360, y_off), cv2.FONT_HERSHEY_SIMPLEX, 0.42, stat_col, 2, cv2.LINE_AA)

    # ─── 4. RIGHT HUD PANEL: 2x2 MIRROR GRID VISUALIZER ───────────────────────
    vis_w, vis_h = 320, 185
    vx = w - vis_w - 15
    vy = h - vis_h - 45
    _draw_mirror_grid_visualizer(frame, vx, vy, vis_w, vis_h, controller, target_pans, target_tilts)

    # ─── 5. BOTTOM REFERENCE BAR ──────────────────────────────────────────────
    _draw_translucent_box(frame, 0, h - 30, w, h, alpha=0.82)
    footer_text = "[Q] Quit System   |   [H] Home All Servos (90 deg, 90 deg)   |   Node 1 (TX): Tracked via ArUco Marker ID 1"
    cv2.putText(frame, footer_text, (15, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_TEXT_MUTED, 1, cv2.LINE_AA)

    return frame
