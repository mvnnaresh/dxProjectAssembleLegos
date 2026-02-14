#include <string>
#include "demo.h"

int main()
{
    const std::string modelPath = "models/ur10e_2f85_scene.xml";
    demo app(modelPath, true);
    if (!app.init())
        return -1;

    app.run(10);
    return 0;
}
/*
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

static bool write_text_file(const std::string& path, const std::string& text)
{
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out.write(text.c_str(), static_cast<std::streamsize>(text.size()));
    return static_cast<bool>(out);
}

static void glfw_error_callback(int error, const char* desc)
{
    std::fprintf(stderr, "GLFW error %d: %s\n", error, desc ? desc : "(null)");
}

int main()
{
    glfwSetErrorCallback(glfw_error_callback);

    if (!glfwInit())
    {
        std::fprintf(stderr, "Failed to initialise GLFW\n");
        return 1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo viewer test", nullptr, nullptr);
    if (!window)
    {
        std::fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return 2;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    const std::string xml = R"XML(
<mujoco model="minimal_drop_test">
  <option timestep="0.002" gravity="0 0 -9.81" integrator="RK4"/>
  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1" rgba="0.8 0.9 0.8 1"/>
    <body name="box" pos="0 0 1">
      <joint name="free" type="free"/>
      <geom name="boxgeom" type="box" size="0.1 0.1 0.1" density="1000" rgba="0.2 0.4 0.8 1"/>
    </body>
  </worldbody>
</mujoco>
)XML";

	const std::string filename = "minimal.xml";
	if (!write_text_file(filename, xml))
	{
		std::fprintf(stderr, "Failed to write %s\n", filename.c_str());
		glfwDestroyWindow(window);
		glfwTerminate();
		return 3;
	}

	char err[1024] = { 0 };
	mjModel* m = mj_loadXML(filename.c_str(), nullptr, err, sizeof(err));
	if (!m)
	{
		std::fprintf(stderr, "mj_loadXML failed: %s\n", err);
		glfwDestroyWindow(window);
		glfwTerminate();
		return 4;
	}

	mjData* d = mj_makeData(m);
	if (!d)
	{
		std::fprintf(stderr, "mj_makeData failed\n");
		mj_deleteModel(m);
		glfwDestroyWindow(window);
		glfwTerminate();
		return 5;
	}

	mjvCamera cam;
	mjvOption opt;
	mjvScene scn;
	mjrContext con;

	mjv_defaultCamera(&cam);
	mjv_defaultOption(&opt);
	mjv_defaultScene(&scn);
	mjr_defaultContext(&con);

	mjv_makeScene(m, &scn, 2000);
	mjr_makeContext(m, &con, mjFONTSCALE_150);

	cam.azimuth = 90.0;
	cam.elevation = -25.0;
	cam.distance = 3.0;
	cam.lookat[0] = 0.0;
	cam.lookat[1] = 0.0;
	cam.lookat[2] = 0.5;

	std::printf("MuJoCo version: %s\n", mj_versionString());

	// Simple real time stepping
	double last_wall = glfwGetTime();
	double sim_accum = 0.0;

	while (!glfwWindowShouldClose(window))
	{
		const double now = glfwGetTime();
		double dt_wall = now - last_wall;
		last_wall = now;

		if (dt_wall > 0.1) dt_wall = 0.1; // avoid huge jumps on pause
		sim_accum += dt_wall;

		const double dt_sim = m->opt.timestep;
		while (sim_accum >= dt_sim)
		{
			mj_step(m, d);
			sim_accum -= dt_sim;
		}

		int w = 0, h = 0;
		glfwGetFramebufferSize(window, &w, &h);
		mjrRect viewport = { 0, 0, w, h };

		mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
		mjr_render(viewport, &scn, &con);

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	mjr_freeContext(&con);
	mjv_freeScene(&scn);
	mj_deleteData(d);
	mj_deleteModel(m);

	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}

*/

