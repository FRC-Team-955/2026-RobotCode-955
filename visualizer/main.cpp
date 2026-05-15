#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/nt/IntegerArrayTopic.hpp>
#include <wpi/nt/StringTopic.hpp>
#include <wpi/nt/StructArrayTopic.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <raylib.h>
#include <raymath.h>

using namespace wpi::nt;
using namespace wpi::math;

#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"

#if defined(PLATFORM_DESKTOP)
	#define GLSL_VERSION            330
#else // PLATFORM_ANDROID, PLATFORM_WEB
	#define GLSL_VERSION            100
#endif

const float ds_x = 8.5f;
const float ds_z = 1.8f;
const Vector3 camera_positions[] = {
	(Vector3){ ds_x, ds_z, 3.07975f }, // Blue 1
	(Vector3){ ds_x, ds_z, 1.25095f }, // Blue 2
	(Vector3){ ds_x, ds_z, -1.8288f }, // Blue 3
	(Vector3){ -ds_x, ds_z, -3.07975f }, // Red 1
	(Vector3){ -ds_x, ds_z, -1.25095f }, // Red 2
	(Vector3){ -ds_x, ds_z, 1.8288f }, // Red 3
};
Vector3 camera_target = (Vector3){ 0.0f, -2.0f, 0.0f };
Vector3 camera_up = (Vector3){ 0.0f, 1.0f, 0.0f };

const float field_length = 16.541f;
const float field_width = 8.069f;

Vector3 wpi_to_raylib(float x, float y, float z) {
	return (Vector3){ .x = field_length / 2.0f - x, .y = z, .z = y - field_width / 2.0f };
}

int main() {
	SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
	InitWindow(1920, 1080, "visualizer");
	SetWindowState(FLAG_WINDOW_RESIZABLE);
	MaximizeWindow();

	// Define the camera to look into our 3d world
	Camera camera = { 0 };
	int camera_position_index = 0;
	camera.position = camera_positions[camera_position_index];                  // Camera position
	camera.target = camera_target;      // Camera looking at point
	camera.up = camera_up;          // Camera up vector (rotation towards target)
	camera.fovy = 80.0f;                                // Camera field-of-view Y
	camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

	DisableCursor();

	Shader shader = LoadShader(
		TextFormat("resources/shaders/glsl%i/lighting.vs", GLSL_VERSION),
		TextFormat("resources/shaders/glsl%i/lighting.fs", GLSL_VERSION)
	);
	// Get some required shader locations
	shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");

	// Ambient light level (some basic lighting)
	int ambient_loc = GetShaderLocation(shader, "ambient");
	float ambient_strength = 0.1f;
	SetShaderValue(shader, ambient_loc, (float[4]){ ambient_strength, ambient_strength, ambient_strength, 1.0f }, SHADER_UNIFORM_VEC4);

	// Add additional lights
	// White point lights
	CreateLight(LIGHT_POINT, (Vector3){ -6.0f, 10.0f, 0.0f }, Vector3Zero(), (Color){ 255, 255, 255, 255 }, shader);
	CreateLight(LIGHT_POINT, (Vector3){ 6.0f, 10.0f, 0.0f }, Vector3Zero(), (Color){ 255, 255, 255, 255 }, shader);
	CreateLight(LIGHT_POINT, (Vector3){ 0.0f, 10.0f, 0.0f }, Vector3Zero(), (Color){ 255, 255, 255, 255 }, shader);
	// Red point lights
	CreateLight(LIGHT_POINT, (Vector3){ -6.0f, 3.0f, 2.5f }, Vector3Zero(), (Color){ 100, 0, 0, 255 }, shader);
	CreateLight(LIGHT_POINT, (Vector3){ -6.0f, 3.0f, -2.5f }, Vector3Zero(), (Color){ 100, 0, 0, 255 }, shader);
	// Blue point lights
	CreateLight(LIGHT_POINT, (Vector3){ 6.0f, 3.0f, 2.5f }, Vector3Zero(), (Color){ 0, 0, 200, 255 }, shader);
	CreateLight(LIGHT_POINT, (Vector3){ 6.0f, 3.0f, -2.5f }, Vector3Zero(), (Color){ 0, 0, 200, 255 }, shader);

	// Load models
	Model field_model = LoadModel("resources/models/field.glb");
	Model robot_model = LoadModel("resources/models/robot.glb");

	// Assign shader to all model materials
	for (int i = 0; i < field_model.materialCount; i++) { field_model.materials[i].shader = shader; }
	for (int i = 0; i < robot_model.materialCount; i++) { robot_model.materials[i].shader = shader; }

	// Start NT client
	auto nt_inst = NetworkTableInstance::GetDefault();
	nt_inst.StartClient("visualizer");
	nt_inst.SetServer("127.0.0.1", 5810);
	PubSubOptions options = { .periodic = 0.02 };
	StringSubscriber status = nt_inst.GetStringTopic("/Multiplayer/Status").Subscribe({}, options);
	IntegerArraySubscriber connected_ids = nt_inst.GetIntegerArrayTopic("/Multiplayer/ConnectedIDs").Subscribe({}, options);
	StructArraySubscriber<Pose2d> robot_poses = nt_inst.GetStructArrayTopic<Pose2d>("/Multiplayer/RobotPoses").Subscribe({}, options);
	StructArraySubscriber<Translation3d> fuel_translations = nt_inst.GetStructArrayTopic<Translation3d>("/Multiplayer/FuelTranslations").Subscribe({}, options);

	SetTargetFPS(80);

	while (!WindowShouldClose()) {
		UpdateCamera(&camera, CAMERA_FREE);

		if (IsKeyPressed(KEY_C)) {
			if (Vector3Equals(camera.position, camera_positions[camera_position_index])) {
				camera_position_index++;
				if (camera_position_index >= sizeof(camera_positions) / sizeof(Vector3)) {
					camera_position_index = 0;
				}
			}
			camera.position = camera_positions[camera_position_index];
			camera.target = camera_target;
			camera.up = camera_up;
		}

		if (IsKeyPressed(KEY_P)) {
			camera.fovy += 10.0f;
		}

		if (IsKeyPressed(KEY_L)) {
			camera.fovy -= 10.0f;
		}

		// Update the shader with the camera view vector (points towards { 0.0f, 0.0f, 0.0f })
		float camera_pos[3] = { camera.position.x, camera.position.y, camera.position.z };
		SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], camera_pos, SHADER_UNIFORM_VEC3);

		BeginDrawing();

			ClearBackground(DARKGRAY);

			BeginMode3D(camera);

				BeginShaderMode(shader);

					DrawModel(field_model, (Vector3){ 0.0f, 0.0f, 0.0f }, 1.0f, WHITE);

					for (Pose2d pose : robot_poses.Get()) {
						DrawModelEx(
							robot_model,
							wpi_to_raylib(pose.X().value(), pose.Y().value(), 0.0f),
							(Vector3){ 0.0f, 1.0f, 0.0f },
							180.0f + pose.Rotation().Degrees().value(),
							(Vector3){ 1.0f, 1.0f, 1.0f },
							WHITE
						);
					}

					for (Translation3d fuel : fuel_translations.Get()) {
						DrawSphere(wpi_to_raylib(fuel.X().value(), fuel.Y().value(), fuel.Z().value()), 0.15f / 2.0f, YELLOW);
					}

				EndShaderMode();

			EndMode3D();

			DrawFPS(10, 10);

			DrawText("Press C to switch camera", 10, 30, 24, WHITE);
			DrawText("Press P/L to adjust FOV", 10, 30 + 32, 24, WHITE);
			DrawText("Press WASD, Space, Ctrl to move camera", 10, 30 + 32 * 2, 24, WHITE);
			DrawText(status.Get("Start robot code simulation").c_str(), 10, 30 + 32 * 3, 24, WHITE);

		EndDrawing();
	}

	CloseWindow();

	return 0;
}
