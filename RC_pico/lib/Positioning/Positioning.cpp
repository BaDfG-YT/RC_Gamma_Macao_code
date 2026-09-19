#pragma once
#include <Arduino.h>

extern float g_ball_angle;
extern int g_ball_area;
extern unsigned long g_last_ball_ms;
extern int g_ball_radius;

extern float g_yellow_angle;
extern int g_yellow_area;
extern unsigned long g_last_yellow_ms;

extern float g_blue_angle;
extern int g_blue_area;
extern unsigned long g_last_blue_ms;

extern bool g_ball_visible;
extern bool g_yellow_visible;
extern bool g_blue_visible;

void pos_init();
void pos_read_cam();
void procces_cam(char *line);

float getYawDeg();
float getYawSignedDeg();

// === Камера ===
void procces_cam(char *line);

// === Лидар (poses, target, drive command) ===
void procces_lidar(char *line);

// Pose from lidar
extern bool  g_pose_valid;
extern float g_pose_x;
extern float g_pose_y;
extern float g_target_dist;
extern float g_target_x;
extern float g_target_y;
extern unsigned long g_last_pose_ms;

// Drive command from lidar
extern float g_cmd_angle;
extern int   g_arrived;
extern float g_robot_yaw;
extern unsigned long g_last_cmd_ms;