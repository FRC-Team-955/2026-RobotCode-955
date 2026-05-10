#include "raylib.h"
#include "raymath.h"

#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"

#if defined(PLATFORM_DESKTOP)
	#define GLSL_VERSION            330
#else // PLATFORM_ANDROID, PLATFORM_WEB
	#define GLSL_VERSION            100
#endif

const float field_length = 16.541f;
const float field_width = 8.069f;

Vector3 wpi_to_raylib(float x, float y, float z) {
    return (Vector3){ .x = field_length - x, .y = z, .z = field_width + y };
}

int main() {
	SetConfigFlags(FLAG_MSAA_4X_HINT);  // Enable Multi Sampling Anti Aliasing 4x (if available)
	InitWindow(1920, 1080, "visualizer");
	SetWindowState(FLAG_WINDOW_RESIZABLE);

	// Define the camera to look into our 3d world
	Camera camera = { 0 };
	camera.position = (Vector3){ 8.5f, 1.5f, 1.2f };    // Camera position
	camera.target = (Vector3){ 0.0f, 3.0f, 0.0f };      // Camera looking at point
	camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
	camera.fovy = 80.0f;                                // Camera field-of-view Y
	camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

	DisableCursor();

	// Load model
	Model model = LoadModel("resources/models/field.glb");
	Vector3 position = { 0.0f, 0.0f, 0.0f };

	Shader shader = LoadShader(
		TextFormat("resources/shaders/glsl%i/lighting.vs", GLSL_VERSION),
		TextFormat("resources/shaders/glsl%i/lighting.fs", GLSL_VERSION)
	);
	// Get some required shader locations
	shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");

	// Ambient light level (some basic lighting)
	int ambient_loc = GetShaderLocation(shader, "ambient");
	float ambient_strength = 1.0f;
	SetShaderValue(shader, ambient_loc, (float[4]){ ambient_strength, ambient_strength, ambient_strength, 1.0f }, SHADER_UNIFORM_VEC4);

	// Add additional lights
	//CreateLight(LIGHT_POINT, (Vector3){ 0.0f, 1000.0f, 0.0f }, (Vector3){ 0.0f, 700.0f, 0.0f }, (Color){ 255, 255, 255, 255 }, shader);
	CreateLight(LIGHT_POINT, (Vector3){ -5.0f, 3.0f, 0.0f }, (Vector3){ -5.0f, 0.0f, 0.0f }, (Color){ 150, 0, 0, 255 }, shader);
	CreateLight(LIGHT_POINT, (Vector3){ 5.0f, 3.0f, 0.0f }, (Vector3){ 5.0f, 0.0f, 0.0f }, (Color){ 0, 0, 150, 255 }, shader);

	// Assign shader to all model materials
	for (int i = 0; i < model.materialCount; i++) {
		model.materials[i].shader = shader;
	}

	SetTargetFPS(80);

	while (!WindowShouldClose()) {
		UpdateCamera(&camera, CAMERA_FREE);

		// Update the shader with the camera view vector (points towards { 0.0f, 0.0f, 0.0f })
		float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
		SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);

		BeginDrawing();

			ClearBackground(DARKGRAY);

			BeginMode3D(camera);

				BeginShaderMode(shader);

					DrawModel(model, position, 1.0f, WHITE);

				EndShaderMode();

			EndMode3D();

			DrawFPS(10, 10);

		EndDrawing();
	}

	CloseWindow();

	return 0;
}
