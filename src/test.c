#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/gl.h>
#include <GL/glx.h>

#include "my_mat_math_5.h"
#include "my_box.h"

/*OpenGL Definitions*/
#define GLX_CONTEXT_MAJOR_VERSION_ARB 0x2091
#define GLX_CONTEXT_MINOR_VERSION_ARB 0x2092
typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);

/*Structure Definitions*/
struct simple_shader_struct
{
	GLuint shaderList[2];
	GLuint program;
	GLuint uniforms[3];
};

struct no_tex_model_struct
{
	GLuint vbo;
	GLuint vao;
	GLuint ebo;
	int num_indices;
	float * vertexPos;	//this is an array of vertex positions for physics
	int num_verts;
};

/*Global Variables*/
struct simple_shader_struct g_shaderInfo;
struct no_tex_model_struct g_boxModel;
struct box_struct g_a_box[2];
struct box_collision_struct g_base_boxHull;	//this is base box hull left-unmodified
struct no_tex_model_struct g_planeModel;
struct box_collision_struct g_base_planeHull;
struct box_struct g_ground_box;
struct box_struct * g_collidingObjects[3];
float g_projection_mat[16];
float g_neg_camera_pos[3];
float g_neg_camera_rot[2]; //0 = rotX, 0 = rotY in degrees
unsigned int g_simulation_step;
int g_simulation_run; //boolean that says is simulation stepping automatically
GLenum g_e;

static int InitGL(unsigned int width, unsigned int height);
static void DrawScene(void);
static int InitGLShader(struct simple_shader_struct * shader_info, char * vert_shader_filename, char * frag_shader_filename);
static char * LoadShaderSource(char * filename);
static void CalculatePerspectiveMatrix(unsigned int width, unsigned int height);
static int InitBoxModel(struct no_tex_model_struct * pmodel);
static int InitPlaneModel(struct no_tex_model_struct * pmodel);
static int InitHull(float * positions, int num_positions, struct box_collision_struct * phull);
static int InitPlaneHull(float * positions, int num_positions, struct box_collision_struct * phull);
static int InitObjPlane(struct box_struct * plane);
static void UpdateHull(struct box_collision_struct * base_hull, struct box_struct * box);
static int CopyHull(struct box_collision_struct * dest, struct box_collision_struct * src);
int FindSeparatingAxis(struct box_struct * boxA, struct box_struct * boxB, struct d_min_struct * d_min);
static int SATCheckDirection(float * s_vec3, float * point_on_plane, struct box_collision_struct * hullA, struct box_collision_struct * hullB, struct d_min_struct * d_min);
static float SATFindSupport(struct box_collision_struct * hull, float * s_vec3, float * point_on_plane);
static int CheckEdgePlane(struct box_struct * boxA, struct box_struct * boxB, float * axis, float * edgeOrigin, struct d_min_struct * d_min);
static int FilterEdgeCheck(struct box_collision_struct * hullA, struct box_collision_struct * hullB, int i_edgeA, int j_edgeB, float * normalB);
static int CreateEdgeContact(struct d_min_struct * d_min, struct box_collision_struct * boxA, struct box_collision_struct * boxB, struct contact_manifold_struct * contact_manifold);
static int CreateFaceContact(struct d_min_struct * d_min, struct box_collision_struct * boxA, struct box_collision_struct * boxB, struct contact_manifold_struct * contact_manifold);
static int InitObjBox(struct box_struct * pbox, float x, float y, float z);
static int DebugInitTriangle(struct no_tex_model_struct * pmodel);
void VehicleConvertDisplacementMat3To4(float * mat3, float * mat4);
void GetElapsedTime(struct timespec * start, struct timespec * end, struct timespec * result);

/*Simulation Functions*/
static void SimulationStep(void);
static void UpdateBoxSimulation(struct box_struct * pbox);
static void CalculateNewBoxVelocity(struct box_struct * pbox, float * sumTorques, float * sumForces);
static void UpdateBoxVelocity(struct box_struct * box);
static void ApplyCollisionImpulses(struct box_struct * boxA, struct box_struct * boxB, struct contact_info_struct * contacts, int num_contacts, int is_b_ground);
static float CalcBaumgarteBias(float penetration);

/*keyboard functions*/
int CheckKey(char * keys_return, int key_bit_index);
static void HandleKeyboardInput(Display * dpy);

int main(int argc, char ** argv)
{
	Display * display=0;
	GLXFBConfig * fbc=0;
	GLXFBConfig chosen_fbc;
	XVisualInfo * vi=0;
	Colormap cmap;
	XSetWindowAttributes swa;
	Window win;
	Atom wmDelete;
	glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;
	GLXContext ctx=0;
	XEvent event;
	unsigned int width = 1024;
	unsigned int height = 768;
	int glxMajor;
	int glxMinor;
	static int visual_attribs[] =
	{
		GLX_X_RENDERABLE, 1,
		GLX_DRAWABLE_TYPE, GLX_WINDOW_BIT,
		GLX_RENDER_TYPE, GLX_RGBA_BIT,
		GLX_X_VISUAL_TYPE, GLX_TRUE_COLOR,
		GLX_RED_SIZE, 8,
		GLX_GREEN_SIZE, 8,
		GLX_BLUE_SIZE, 8,
		GLX_ALPHA_SIZE, 8,
		GLX_DEPTH_SIZE, 24,
		GLX_DOUBLEBUFFER, 1,
		None
	};
	int context_attribs[] =
	{
		GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
		GLX_CONTEXT_MINOR_VERSION_ARB, 3,
		None
	};
	struct timespec curr_time;
	struct timespec diff;
	struct timespec last_drawcall;
	struct timespec last_simulatecall;
	const struct timespec diff_simulate = {0, 16000000}; //~60Hz
	const struct timespec diff_drawcall = {0, 16000000};
	int fbcount;
	int running;
	int r;

	display = XOpenDisplay(0);
	if(display == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}

	r = glXQueryVersion(display, &glxMajor, &glxMinor);
	if(r == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}

	if((glxMajor == 1) && (glxMinor < 3))
	{
		printf("%s: error. need glx 1.3\n", __func__);
		return 0;
	}

	fbc = glXChooseFBConfig(display, DefaultScreen(display), visual_attribs, &fbcount);
	if(fbc == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}
	printf("found %d frame buffer configurations.\n", fbcount);

	//screw it just pick the first one.
	chosen_fbc = fbc[0];

	//free the FBConfig list
	XFree(fbc);

	vi = glXGetVisualFromFBConfig(display, chosen_fbc);

	//Create colormap
	cmap = XCreateColormap(display, RootWindow(display, vi->screen), vi->visual, AllocNone);
	swa.colormap = cmap;
	swa.background_pixmap = 0;
	swa.border_pixel = 0;
	swa.event_mask = ExposureMask | StructureNotifyMask;

	//Create the window
	win = XCreateWindow(display,
			RootWindow(display, vi->screen),
			0, 0, width, height, 0, vi->depth, InputOutput,
			vi->visual,
			CWBorderPixel | CWColormap | CWEventMask, &swa);
	if(!win)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}

	XFree(vi);

	//this fixes an XIO fatal error when closing
	wmDelete = XInternAtom(display, "WM_DELETE_WINDOW", True);
	XSetWMProtocols(display, win, &wmDelete, 1);

	//Get the GLX context
	glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc)glXGetProcAddressARB("glXCreateContextAttribsARB");
	if(glXCreateContextAttribsARB == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}
	ctx = glXCreateContextAttribsARB(display, chosen_fbc, 0, True, context_attribs);
	if(ctx == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}
	glXMakeCurrent(display, win, ctx);

	//print if we are direct rendering
	if(glXIsDirect(display, ctx))
	{
		printf("direct rendering enabled.\n");
	}
	else
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}
	printf("OpenGL version: %s\n", glGetString(GL_VERSION));
	printf("GL_SHADING_LANGUAGE_VERSION: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));

	//Initialize OpenGL objects and shaders
	running = InitGL(width, height);

	printf("setup complete, entering message loop.\n");
	XMapRaised(display, win);

	//get a starting point for time
	clock_gettime(CLOCK_MONOTONIC, &last_drawcall);
	clock_gettime(CLOCK_MONOTONIC, &last_simulatecall);
	clock_gettime(CLOCK_MONOTONIC, &curr_time);

	while(running)
	{
		while(XPending(display) > 0)
		{
			XNextEvent(display, &event);
			if(event.type == Expose)
			{
				DrawScene();
				glXSwapBuffers(display, win);
			}
			if(event.type == ClientMessage)
			{
				running = 0;
				break;
			}
		}
		
		//get the current time and check to see if the enough time has passed
		//to call update again
		clock_gettime(CLOCK_MONOTONIC, &curr_time);
		GetElapsedTime(&last_simulatecall, &curr_time, &diff);
		if(diff.tv_sec > diff_simulate.tv_sec || (diff.tv_sec == diff_simulate.tv_sec && diff.tv_nsec >= diff_simulate.tv_nsec))
		{
			HandleKeyboardInput(display);
			if(g_simulation_run == 1)
				SimulationStep();
		}

		clock_gettime(CLOCK_MONOTONIC, &curr_time);
		GetElapsedTime(&last_drawcall, &curr_time, &diff);
		if(diff.tv_sec > diff_drawcall.tv_sec || (diff.tv_sec == diff_drawcall.tv_sec && diff.tv_nsec >= diff_drawcall.tv_nsec))
		{
			DrawScene();
			glXSwapBuffers(display, win);
		}
	}

	glXMakeCurrent(display, None, 0);
	glXDestroyContext(display, ctx);
	XCloseDisplay(display);

	return 0;
}

static int InitGL(unsigned int width, unsigned int height)
{
	float temp_vec[3];
	float temp_q[4];
	int r;

	glViewport(0,				//lower-left corner x
			0,					//lower-left corner y
			(GLsizei)width,		//width of viewport 
			(GLsizei)height);	//height of viewport

	CalculatePerspectiveMatrix(width, height);
	//g_neg_camera_rot[1] = 180.0f;
	g_neg_camera_pos[0] = -0.5f;
	g_neg_camera_pos[1] = -2.0f;

	r = InitGLShader(&g_shaderInfo, "model.vert", "model.frag");
	if(!r)
		return 0;

	//Setup the box model
	r = InitBoxModel(&g_boxModel);
	//r = DebugInitTriangle(&g_boxModel);
	if(r == 0)
		return 0;

	r = InitPlaneModel(&g_planeModel);
	if(r == 0)
		return 0;

	//Setup physics for boxes
	InitObjBox(g_a_box, 0.0f, 3.0f, -10.0f);
	InitObjBox((g_a_box+1), 0.0f, 1.0f, -10.0f);

	//Setup the scene a little bit
	//make box0 slowly travel downward
	g_a_box[0].linearVel[0] = 0.0f;
	g_a_box[0].linearVel[1] = -0.001f;
	g_a_box[0].linearVel[2] = 0.0f;
	g_a_box[0].pos[0] += 0.7f; //move box0 a little to the right so it hits box B funny
	g_a_box[0].pos[2] -= 0.5f;

	//this bit causes box to rotate
	temp_vec[0] = 0.0f;
	temp_vec[1] = 0.0f;
	temp_vec[2] = 1.0f;
	qCreate(temp_q, temp_vec, 45.0f);
	g_a_box[0].orientationQ[0] = temp_q[0];
	g_a_box[0].orientationQ[1] = temp_q[1];
	g_a_box[0].orientationQ[2] = temp_q[2];
	g_a_box[0].orientationQ[3] = temp_q[3];
	qConvertToMat3(temp_q, g_a_box[0].orientation);

	r = InitHull(g_boxModel.vertexPos, g_boxModel.num_verts, &g_base_boxHull);
	if(r == 0)
		return 0;
	r = CopyHull(&(g_a_box->hull), &g_base_boxHull);
	if(r == 0)
		return 0;
	r = CopyHull(&(g_a_box[1].hull), &g_base_boxHull);
	if(r == 0)
		return 0;
	UpdateHull(&g_base_boxHull, g_a_box);
	UpdateHull(&g_base_boxHull, (g_a_box+1));

	r = InitObjPlane(&g_ground_box);
	if(r == 0)
		return 0;
	r = InitPlaneHull(g_planeModel.vertexPos, g_planeModel.num_verts, &g_ground_box.hull);
	if(r == 0)
		return 0;

	//initialize this array for SimulationStep() so it can easily
	//iterate through all hulls
	g_collidingObjects[0] = &(g_a_box[0]);
	g_collidingObjects[1] = &(g_a_box[1]);
	g_collidingObjects[2] = &(g_ground_box);

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);

	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glDepthFunc(GL_LEQUAL);
	glDepthRange(0.0f, 1.0f);

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClearDepth(1.0f);

	//g_e = glGetError();
	//printf("glGetError=0x%X\n", g_e);

	return 1;
}

static void DrawScene(void)
{
	float camera_mat[16];
	float camera_translate_mat[16];
	float camera_rotate_mat_Y[16];
	float camera_rotate_mat_X[16];
	float camera_rotate_mat[16];
	float model_rotate_mat[16];
	float model_translate_mat[16];
	float model_mat[16];
	float model_to_camera_mat[16];
	float normal_mat[9];
	int i;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//g_e = glGetError();
	//printf("glGetError=0x%X\n", g_e);

	mmMakeIdentityMatrix(camera_mat);
	mmTranslateMatrix(camera_translate_mat, g_neg_camera_pos[0], g_neg_camera_pos[1], g_neg_camera_pos[2]);
	mmRotateAboutY(camera_rotate_mat_Y, g_neg_camera_rot[1]);
	mmRotateAboutX(camera_rotate_mat_X, g_neg_camera_rot[0]);
	mmMultiplyMatrix4x4(camera_rotate_mat_X, camera_rotate_mat_Y, camera_rotate_mat);

	//We want to translate and then rotate the world by the opposite of the
	//camera's position and rotation. So we want to translate then rotate.
	//Thus since we are using column major matrices the order of our operations
	//needs to be: rotate then translate.
	mmMultiplyMatrix4x4(camera_rotate_mat, camera_translate_mat, camera_mat);

	glUseProgram(g_shaderInfo.program);

	//draw ground plane
	glBindVertexArray(g_planeModel.vao);
	mmMakeIdentityMatrix(model_mat);
	mmMultiplyMatrix4x4(camera_mat, model_mat, model_to_camera_mat);
	mmConvertMat4toMat3(model_mat, normal_mat);
	glUniformMatrix3fv(g_shaderInfo.uniforms[2], 1, GL_FALSE, normal_mat);
	glUniformMatrix4fv(g_shaderInfo.uniforms[1], 1, GL_FALSE, model_to_camera_mat);
	glDrawElements(GL_TRIANGLES,
			g_planeModel.num_indices,
			GL_UNSIGNED_BYTE,
			0);

	//draw boxes
	glBindVertexArray(g_boxModel.vao);
	for(i = 0; i < 2; i++)
	{
		VehicleConvertDisplacementMat3To4(g_a_box[i].orientation, model_rotate_mat);
		mmTranslateMatrix(model_translate_mat, g_a_box[i].pos[0], g_a_box[i].pos[1], g_a_box[i].pos[2]);
		mmMultiplyMatrix4x4(model_translate_mat, model_rotate_mat, model_mat);
		mmMultiplyMatrix4x4(camera_mat, model_mat, model_to_camera_mat);

		mmConvertMat4toMat3(model_mat, normal_mat);

			glUniformMatrix3fv(g_shaderInfo.uniforms[2], 1, GL_FALSE, normal_mat);
			glUniformMatrix4fv(g_shaderInfo.uniforms[1], 1, GL_FALSE, model_to_camera_mat);
			glDrawElements(GL_TRIANGLES,
				g_boxModel.num_indices,
				GL_UNSIGNED_BYTE,
				0);
	}
	glBindVertexArray(0);
	glUseProgram(0);
	

	//g_e = glGetError();
	//printf("glGetError=0x%X\n", g_e);
}

static int InitGLShader(struct simple_shader_struct * shader_info, char * vert_shader_filename, char * frag_shader_filename)
{
	char * vertexShaderString=0;
	char * fragmentShaderString=0;
	GLint status;
	GLint infoLogLength;
	GLchar * strInfoLog=0;
	int r;

	vertexShaderString = LoadShaderSource(vert_shader_filename);
	if(vertexShaderString == 0)
		return 0;
	shader_info->shaderList[0] = glCreateShader(GL_VERTEX_SHADER);
	printf("created vertex shader: %d\n", shader_info->shaderList[0]);
	glShaderSource(shader_info->shaderList[0], 1, &vertexShaderString, 0);
	glCompileShader(shader_info->shaderList[0]);
	glGetShaderiv(shader_info->shaderList[0], GL_COMPILE_STATUS, &status);
	free(vertexShaderString);

	if(status == GL_FALSE)
	{
		glGetShaderiv(shader_info->shaderList[0], GL_INFO_LOG_LENGTH, &infoLogLength);
		strInfoLog = (GLchar*)malloc(infoLogLength+1);
		if(strInfoLog == 0)
		{
			printf("%s: error line %d\n", __func__, __LINE__);
			return 0;
		}
		glGetShaderInfoLog(shader_info->shaderList[0], infoLogLength, 0, strInfoLog);
		printf("%s: compile failure in shader(%d):\n%s\n", __func__, shader_info->shaderList[0], strInfoLog);
		free(strInfoLog);
		return 0;
	}

	//So the vertex shader is compiled. Now compile the fragment shader.
	status = 0;
	fragmentShaderString = LoadShaderSource(frag_shader_filename);
	if(fragmentShaderString == 0)
		return 0;
	shader_info->shaderList[1] = glCreateShader(GL_FRAGMENT_SHADER);
	printf("created fragment shader: %d\n", shader_info->shaderList[1]);
	glShaderSource(shader_info->shaderList[1], 1, &fragmentShaderString, 0);
	glCompileShader(shader_info->shaderList[1]);
	glGetShaderiv(shader_info->shaderList[1], GL_COMPILE_STATUS, &status);
	free(fragmentShaderString);

	if(status == GL_FALSE)
	{
		glGetShaderiv(shader_info->shaderList[1], GL_INFO_LOG_LENGTH, &infoLogLength);
		strInfoLog = (GLchar*)malloc(infoLogLength+1);
		if(strInfoLog == 0)
		{
			printf("%s: error line %d\n", __func__, __LINE__);
			return 0;
		}
		glGetShaderInfoLog(shader_info->shaderList[1], infoLogLength, 0, strInfoLog);
		printf("%s: compile failure in shader(%d):\n%s\n", __func__, shader_info->shaderList[1], strInfoLog);
		free(strInfoLog);
		return 0;
	}

	//link the gl program
	status = 0;
	shader_info->program = glCreateProgram();
	if(shader_info->program == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}
	glAttachShader(shader_info->program, shader_info->shaderList[0]);
	glAttachShader(shader_info->program, shader_info->shaderList[1]);
	glLinkProgram(shader_info->program);
	glGetProgramiv(shader_info->program, GL_LINK_STATUS, &status);
	if(status == GL_FALSE)
	{
		glGetProgramiv(shader_info->program, GL_INFO_LOG_LENGTH, &infoLogLength);
		strInfoLog = (GLchar*)malloc(infoLogLength+1);
		if(strInfoLog == 0)
		{
			printf("%s: error line %d\n", __func__, __LINE__);
			return 0;
		}
		glGetProgramInfoLog(shader_info->program, infoLogLength, 0, strInfoLog);
		printf("%s: program %d link failure:\n%s\n", __func__, shader_info->program, strInfoLog);
		free(strInfoLog);
		return 0;
	}

	glDetachShader(shader_info->program, shader_info->shaderList[0]);
	glDetachShader(shader_info->program, shader_info->shaderList[1]);

	glUseProgram(shader_info->program);
	
	//Setup uniforms

	//get location of projection matrix
	shader_info->uniforms[0] = glGetUniformLocation(shader_info->program, "projectionMatrix");
	if(shader_info->uniforms[0] == -1)
	{
		printf("%s: error. failed to get uniform location of %s\n", __func__, "projectionMatrix");
		return 0;
	}
	glUniformMatrix4fv(shader_info->uniforms[0], 1, GL_FALSE, g_projection_mat);

	//get location of modelToCameraMatrix
	shader_info->uniforms[1] = glGetUniformLocation(shader_info->program, "modelToCameraMatrix");
	if(shader_info->uniforms[1] == -1)
	{
		printf("%s: error. failed to get uniform location of %s\n", __func__, "modelToCameraMatrix");
		return 0;
	}

	//get location of normalRotMatrix
	shader_info->uniforms[2] = glGetUniformLocation(shader_info->program, "normalRotMatrix");
	if(shader_info->uniforms[2] == -1)
	{
		printf("%s: error. failed to get uniform location of %s\n", __func__, "normalRotMatrix");
		return 0;
	}

	return 1;
}

static char * LoadShaderSource(char * filename)
{
	FILE * pSrcFile=0;
	int i;
	int shader_src_len;
	char * shader_source=0;
	
	pSrcFile = fopen(filename, "r");
	if(pSrcFile == 0)
	{
		printf("LoadShaderSource: error. could not open %s\n", filename);
		return 0;
	}
	
	//count how big the vertex shader source is in bytes
	i = 0;
	while(feof(pSrcFile) == 0)
	{
		fgetc(pSrcFile);
		i++;
	}
	
	//this will make i one greater than the end of the file, but this is ok
	//because the resulting string of the file contents needs to be null-terminated anyway
	rewind(pSrcFile);
	shader_src_len = i;
	shader_source = (char*)malloc(shader_src_len);
	if(shader_source == 0)
	{
		printf("LoadShaderSource: error. malloc() failed for shader_source string.\n");
		fclose(pSrcFile);
		return 0;
	}
	i = 0;
	while(feof(pSrcFile) == 0)
	{
		shader_source[i] = (char)fgetc(pSrcFile);
		i++;
	}
	shader_source[(i-1)] = '\x00';
	fclose(pSrcFile);
	
	return shader_source;
}

static void CalculatePerspectiveMatrix(unsigned int width, unsigned int height)
{
	float y_scale;
	float x_scale;
	float fzNear;
	float fzFar;

	fzNear = 1.0f;
	fzFar = 200.0f;

	memset(g_projection_mat, 0, sizeof(float)*16);
	y_scale = 1.0f/(tanf((M_PI/180.0f)*(45.0f/2.0f)));
	x_scale = y_scale/((float)width/(float)height);
	g_projection_mat[0] = x_scale;
	g_projection_mat[5] = y_scale;
	g_projection_mat[10] = (fzFar + fzNear)/(fzNear - fzFar);
	g_projection_mat[14] = (2*fzFar*fzNear)/(fzNear - fzFar);
	g_projection_mat[11] = -1.0f;
}

static int DebugInitTriangle(struct no_tex_model_struct * pmodel)
{
	float * vertData=0;
	char * indices=0;

	memset(pmodel, 0, sizeof(struct no_tex_model_struct));

	pmodel->num_verts=3;

	vertData = (float*)malloc(3*6*sizeof(float));
	if(vertData == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}

	indices = (char*)malloc(3);
	if(indices == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}

	vertData[0] = 1.0f;
	vertData[1] = 0.0f;
	vertData[2] = 0.0f;

	vertData[3] = 0.0f;
	vertData[4] = 0.0f;
	vertData[5] = 1.0f;

	vertData[6] = -1.0f;
	vertData[7] = 0.0f;
	vertData[8] = 0.0f;

	vertData[9] = 0.0f;
	vertData[10] = 0.0f;
	vertData[11] = 1.0f;

	vertData[12] = 0.0f;
	vertData[13] = 1.0f;
	vertData[14] = 0.0f;

	vertData[15] = 0.0f;
	vertData[16] = 0.0f;
	vertData[17] = 1.0f;

	indices[0] = 0;
	indices[1] = 2;
	indices[2] = 1;

	glGenBuffers(1, &(pmodel->vbo));
	glBindBuffer(GL_ARRAY_BUFFER, pmodel->vbo);
	glBufferData(GL_ARRAY_BUFFER,
			(GLsizeiptr)(3*6*sizeof(float)),
			vertData,
			GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &(pmodel->ebo));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pmodel->ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,
			(GLsizeiptr)(3),
			indices,
			GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	pmodel->num_indices = 3;

	glGenVertexArrays(1, &(pmodel->vao));
	glBindVertexArray(pmodel->vao);
	glBindBuffer(GL_ARRAY_BUFFER, pmodel->vbo);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), 0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (GLvoid*)(3*sizeof(float)));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pmodel->ebo);
	glBindVertexArray(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	return 1;
}

static int InitBoxModel(struct no_tex_model_struct * pmodel)
{
	float * vertData=0;
	unsigned char * indices=0;
	float faceNormal[3];
	int lenVertData;

	memset(pmodel, 0, sizeof(struct no_tex_model_struct));

	pmodel->vertexPos = (float*)malloc(24*sizeof(float));	//8 verts * 3 floats-per-vert
	if(pmodel->vertexPos == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}

	//vertex data.
	//6 faces. 4 vertices per face.
	lenVertData = 24; //num vertices in vertData
	vertData = (float*)malloc(24*6*sizeof(float)); //24 unique vertices * 6 floats per vert. 3 pos + 3 normal
	if(vertData == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}

	//allocate indices.
	//6 faces, 2 tri's per face, 3 indices per tri: 6*2*3
	pmodel->num_indices = 36;
	indices = (unsigned char*)malloc(36);
	if(indices == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}

	//keep the vertex positions around for collision detection.
	pmodel->num_verts = 8; //this is used for physics and not by DrawScene()
	//+x,-y,-z	vert 0
	pmodel->vertexPos[0] = 0.5f;
	pmodel->vertexPos[1] = -0.5f;
	pmodel->vertexPos[2] = -0.5f;

	//+x,-y,+z	vert 1
	pmodel->vertexPos[3] = 0.5f;
	pmodel->vertexPos[4] = -0.5f;
	pmodel->vertexPos[5] = 0.5f;

	//-x,-y,+z	vert 2
	pmodel->vertexPos[6] = -0.5f;
	pmodel->vertexPos[7] = -0.5f;
	pmodel->vertexPos[8] = 0.5f;

	//-x,-y,-z	vert 3
	pmodel->vertexPos[9] = -0.5f;
	pmodel->vertexPos[10] = -0.5f;
	pmodel->vertexPos[11] = -0.5f;

	//+x,+y,-z	vert 4
	pmodel->vertexPos[12] = 0.5f;
	pmodel->vertexPos[13] = 0.5f;
	pmodel->vertexPos[14] = -0.5f;

	//+x,+y,+z	vert 5
	pmodel->vertexPos[15] = 0.5f;
	pmodel->vertexPos[16] = 0.5f;
	pmodel->vertexPos[17] = 0.5f;

	//-x,+y,+z	vert 6
	pmodel->vertexPos[18] = -0.5f;
	pmodel->vertexPos[19] = 0.5f;
	pmodel->vertexPos[20] = 0.5f;

	//-x,+y,-z	vert 7
	pmodel->vertexPos[21] = -0.5f;
	pmodel->vertexPos[22] = 0.5f;
	pmodel->vertexPos[23] = -0.5f;

	//interleave the vertex data
	//-y face.
	faceNormal[0] = 0.0f;
	faceNormal[1] = -1.0f;
	faceNormal[2] = 0.0f;
	memcpy(vertData, (pmodel->vertexPos), 3*sizeof(float));	//pos 0, vert 0
	memcpy((vertData+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*1)), (pmodel->vertexPos+(3*1)), 3*sizeof(float)); //pos 1, vert 1
	memcpy((vertData+(6*1)+3), faceNormal, 3*sizeof(float));
	
	memcpy((vertData+(6*2)), (pmodel->vertexPos+(3*2)), 3*sizeof(float)); //pos 2, vert 2
	memcpy((vertData+(6*2)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*3)), (pmodel->vertexPos+(3*3)), 3*sizeof(float)); //pos 3, vert 3
	memcpy((vertData+(6*3)+3), faceNormal, 3*sizeof(float));

	//+y face
	faceNormal[0] = 0.0f;
	faceNormal[1] = 1.0f;
	faceNormal[2] = 0.0f;
	memcpy((vertData+(6*4)), (pmodel->vertexPos+(3*4)), 3*sizeof(float)); //pos 4, vert 4
	memcpy((vertData+(6*4)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*5)), (pmodel->vertexPos+(3*7)), 3*sizeof(float)); //pos 7, vert 5
	memcpy((vertData+(6*5)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*6)), (pmodel->vertexPos+(3*6)), 3*sizeof(float)); //pos 6, vert 6
	memcpy((vertData+(6*6)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*7)), (pmodel->vertexPos+(3*5)), 3*sizeof(float)); //pos 5, vert 7
	memcpy((vertData+(6*7)+3), faceNormal, 3*sizeof(float));

	//-z face
	faceNormal[0] = 0.0f;
	faceNormal[1] = 0.0f;
	faceNormal[2] = -1.0f;
	memcpy((vertData+(6*8)), (pmodel->vertexPos), 3*sizeof(float)); //pos 0, vert 8
	memcpy((vertData+(6*8)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*9)), (pmodel->vertexPos+(3*3)), 3*sizeof(float)); //pos 3, vert 9
	memcpy((vertData+(6*9)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*10)), (pmodel->vertexPos+(3*7)), 3*sizeof(float)); //pos 7, vert 10
	memcpy((vertData+(6*10)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*11)), (pmodel->vertexPos+(3*4)), 3*sizeof(float)); //pos 4, vert 11
	memcpy((vertData+(6*11)+3), faceNormal, 3*sizeof(float));

	//+z face
	faceNormal[0] = 0.0f;
	faceNormal[1] = 0.0f;
	faceNormal[2] = 1.0f;
	memcpy((vertData+(6*12)), (pmodel->vertexPos+(3*2)), 3*sizeof(float)); //pos 2, vert 12
	memcpy((vertData+(6*12)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*13)), (pmodel->vertexPos+(3*1)), 3*sizeof(float)); //pos 1, vert 13
	memcpy((vertData+(6*13)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*14)), (pmodel->vertexPos+(3*5)), 3*sizeof(float)); //pos 5, vert 14
	memcpy((vertData+(6*14)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*15)), (pmodel->vertexPos+(3*6)), 3*sizeof(float)); //pos 6, vert 15
	memcpy((vertData+(6*15)+3), faceNormal, 3*sizeof(float));

	//-x face
	faceNormal[0] = -1.0f;
	faceNormal[1] = 0.0f;
	faceNormal[2] = 0.0f;
	memcpy((vertData+(6*16)), (pmodel->vertexPos+(3*3)), 3*sizeof(float)); //pos 3, vert 16
	memcpy((vertData+(6*16)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*17)), (pmodel->vertexPos+(3*2)), 3*sizeof(float)); //pos 2, vert 17
	memcpy((vertData+(6*17)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*18)), (pmodel->vertexPos+(3*6)), 3*sizeof(float)); //pos 6, vert 18
	memcpy((vertData+(6*18)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*19)), (pmodel->vertexPos+(3*7)), 3*sizeof(float)); //pos 7, vert 19
	memcpy((vertData+(6*19)+3), faceNormal, 3*sizeof(float));

	//+x face
	faceNormal[0] = 1.0f;
	faceNormal[1] = 0.0f;
	faceNormal[2] = 0.0f;
	memcpy((vertData+(6*20)), (pmodel->vertexPos+(3*1)), 3*sizeof(float)); //pos 1, vert 20
	memcpy((vertData+(6*20)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*21)), (pmodel->vertexPos), 3*sizeof(float)); //pos 0, vert 21
	memcpy((vertData+(6*21)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*22)), (pmodel->vertexPos+(3*4)), 3*sizeof(float)); //pos 4, vert 22
	memcpy((vertData+(6*22)+3), faceNormal, 3*sizeof(float));

	memcpy((vertData+(6*23)), (pmodel->vertexPos+(3*5)), 3*sizeof(float)); //pos 5, vert 23
	memcpy((vertData+(6*23)+3), faceNormal, 3*sizeof(float));
	
	//set indices
	//-y face
	indices[0] = 2;
	indices[1] = 3;
	indices[2] = 0;
	indices[3] = 2;
	indices[4] = 0;
	indices[5] = 1;

	//+y face
	indices[6] = 5;
	indices[7] = 6;
	indices[8] = 7;
	indices[9] = 7;
	indices[10] = 4;
	indices[11] = 5;

	//-z face
	indices[12] = 9;
	indices[13] = 10;
	indices[14] = 11;
	indices[15] = 9;
	indices[16] = 11;
	indices[17] = 8;

	//+z face
	indices[18] = 13;
	indices[19] = 14;
	indices[20] = 15;
	indices[21] = 13;
	indices[22] = 15;
	indices[23] = 12;

	//-x face
	indices[24] = 17;
	indices[25] = 18;
	indices[26] = 19;
	indices[27] = 17;
	indices[28] = 19;
	indices[29] = 16;

	//+x face
	indices[30] = 21;
	indices[31] = 22;
	indices[32] = 23;
	indices[33] = 21;
	indices[34] = 23;
	indices[35] = 20;

	//Now setup the OpenGL stuff.
	
	//setup the VBO
	glGenBuffers(1, &(pmodel->vbo));
	glBindBuffer(GL_ARRAY_BUFFER, pmodel->vbo);
	glBufferData(GL_ARRAY_BUFFER,								//target buffer.
			(GLsizeiptr)(lenVertData*6*sizeof(float)),			//size in bytes.
			vertData,											//addr of data.
			GL_STATIC_DRAW);									//usage hint.
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	//setup the EBO
	glGenBuffers(1, &(pmodel->ebo));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pmodel->ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,
			(GLsizeiptr)(pmodel->num_indices),	//36
			indices,
			GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	//setup the VAO
	glGenVertexArrays(1, &(pmodel->vao));
	glBindVertexArray(pmodel->vao);
	glBindBuffer(GL_ARRAY_BUFFER, pmodel->vbo);
	glEnableVertexAttribArray(0);	//vertex position
	glEnableVertexAttribArray(1);	//vertex normal
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), 0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (GLvoid*)(3*sizeof(float)));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pmodel->ebo);
	glBindVertexArray(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	
	return 1;//success
}

static int InitObjBox(struct box_struct * pbox, float x, float y, float z)
{
	float momentOfInertia[9];
	float inertiaTerm;

	memset(pbox, 0, sizeof(struct box_struct));

	pbox->pos[0] = x;
	pbox->pos[1] = y;
	pbox->pos[2] = z;

	pbox->orientationQ[0] = 0.0f; //x
	pbox->orientationQ[1] = 0.0f; //y
	pbox->orientationQ[2] = 0.0f; //z
	pbox->orientationQ[3] = 1.0f; //w
	qConvertToMat3(pbox->orientationQ, pbox->orientation);

	pbox->mass = 1.0f;

	inertiaTerm = (1.0f/12.0f)*pbox->mass*(1.0f + 1.0f);	//assume box width, height both == 1
	pbox->imomentOfInertia[0] = 1.0f/inertiaTerm;
	pbox->imomentOfInertia[4] = 1.0f/inertiaTerm;
	pbox->imomentOfInertia[8] = 1.0f/inertiaTerm;

	//TODO: Remove this debug. Check rotation
	//pbox->angularMomentum[1] = 0.0001f;

	return 1;//success
}

/*
borrow the box_struct, since the collision functions pull out some
velocity information from both objects even if one of them is ground.
*/
static int InitObjPlane(struct box_struct * plane)
{
	memset(plane, 0, sizeof(struct box_struct));

	plane->orientationQ[0] = 0.0f; //x
	plane->orientationQ[1] = 0.0f; //y
	plane->orientationQ[2] = 0.0f; //z
	plane->orientationQ[3] = 1.0f; //w
	qConvertToMat3(plane->orientationQ, plane->orientation);

	return 1; //succes
}

static int InitPlaneModel(struct no_tex_model_struct * pmodel)
{
	float * vertData = 0;
	unsigned char * indices=0;
	float halfSize = 20.0f;
	float normal[3] = {0.0f, 1.0f, 0.0f};

	memset(pmodel, 0, sizeof(struct no_tex_model_struct));

	pmodel->vertexPos = (float*)malloc(4*3*sizeof(float));
	if(pmodel->vertexPos == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}
	
	//1 face. 4 verts
	vertData = (float*)malloc(4*6*sizeof(float)); //4 unique vertices * 6 floats-per-vert
	if(vertData == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}

	//allocate indices
	pmodel->num_indices = 6; //2 triangles to make 1 quad.
	indices = (unsigned char*)malloc(6);
	if(indices == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}

	//set vertex positions. these are kept around for collision detection.
	pmodel->num_verts = 4;

	//+x,-z		0
	pmodel->vertexPos[0] = halfSize;
	pmodel->vertexPos[1] = 0.0f;
	pmodel->vertexPos[2] = -1.0f*halfSize;

	//-x,-z		1
	pmodel->vertexPos[3] = -1.0f*halfSize;
	pmodel->vertexPos[4] = 0.0f;
	pmodel->vertexPos[5] = -1.0f*halfSize;

	//-x,+z		2
	pmodel->vertexPos[6] = -1.0f*halfSize;
	pmodel->vertexPos[7] = 0.0f;
	pmodel->vertexPos[8] = halfSize;

	//+x,+z		3
	pmodel->vertexPos[9] = halfSize;
	pmodel->vertexPos[10] = 0.0f;
	pmodel->vertexPos[11] = halfSize;

	//interleave the vertex data
	memcpy(vertData, pmodel->vertexPos, 3*sizeof(float));
	memcpy((vertData+3), normal, 3*sizeof(float));

	memcpy((vertData+(6*1)), (pmodel->vertexPos+(3*1)), 3*sizeof(float));
	memcpy((vertData+(6*1)+3), normal, 3*sizeof(float));

	memcpy((vertData+(6*2)), (pmodel->vertexPos+(3*2)), 3*sizeof(float));
	memcpy((vertData+(6*2)+3), normal, 3*sizeof(float));

	memcpy((vertData+(6*3)), (pmodel->vertexPos+(3*3)), 3*sizeof(float));
	memcpy((vertData+(6*3)+3), normal, 3*sizeof(float));

	//set indices
	indices[0] = 0;
	indices[1] = 1;
	indices[2] = 2;
	indices[3] = 0;
	indices[4] = 2;
	indices[5] = 3;

	//setup the VBO
	glGenBuffers(1, &(pmodel->vbo));
	glBindBuffer(GL_ARRAY_BUFFER, pmodel->vbo);
	glBufferData(GL_ARRAY_BUFFER,
			(GLsizeiptr)(4*6*sizeof(float)),
			vertData,
			GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	//setup the EBO
	glGenBuffers(1, &(pmodel->ebo));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pmodel->ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,
			(GLsizeiptr)(pmodel->num_indices),	//6
			indices,
			GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	//setup the VAO
	glGenVertexArrays(1, &(pmodel->vao));
	glBindVertexArray(pmodel->vao);
	glBindBuffer(GL_ARRAY_BUFFER, pmodel->vbo);
	glEnableVertexAttribArray(0);	//vertex position
	glEnableVertexAttribArray(1);	//vertex normal
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), 0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (GLvoid*)(3*sizeof(float)));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pmodel->ebo);
	glBindVertexArray(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	return 1;
}

static void SimulationStep(void)
{
	struct box_struct * boxA=0;
	struct box_struct * boxB=0;
	struct contact_manifold_struct contact_manifold;
	struct d_min_struct d_min;
	float externalTorque[3] = {0.0f, 0.0f, 0.0f};
	float externalForce[3] = {0.0f, 0.0f, 0.0f};
	int num_hulls=3;
	int index_of_ground=2;
	int k;
	int i;
	int j;
	int r;

	memset(&d_min, 0, sizeof(struct d_min_struct));
	memset(&contact_manifold, 0, sizeof(struct contact_manifold_struct));

	//for now just apply 0 forces. This needs to be called otherwise velocities
	//will get zero'd.
	CalculateNewBoxVelocity(&(g_a_box[1]), externalTorque, externalForce);
	//apply some force to box0
	externalForce[1] = -0.00001f;
	CalculateNewBoxVelocity(&(g_a_box[0]), externalTorque, externalForce);
	UpdateBoxVelocity(&(g_a_box[0]));

	//loop through all possible hull collision combinations
	i = 0;
	j = 1;
	for(k = 0; k < num_hulls; k++)
	{
		boxA = g_collidingObjects[i];
		boxB = g_collidingObjects[j];

		//r = FindSeparatingAxis(&(g_a_box[0]), &(g_a_box[1]), &d_min);
		r = FindSeparatingAxis(boxA, boxB, &d_min);
		if(r == 0)	//a separating axis was not found
		{
			contact_manifold.overlappedBoxes[0] = boxA;
			contact_manifold.overlappedBoxes[1] = boxB;

			if(d_min.source == 0)	//if source of s_min is a face
			{
				CreateFaceContact(&d_min,
					&(boxA->hull),
					//&(g_a_box[1].hull),
					&(boxB->hull),
					&contact_manifold);
			}
			if(d_min.source == 1)	//if source of s_min is an edge
			{
				//CreateEdgeContact(&d_min, &(g_a_box[0].hull), &(g_a_box[1].hull), contacts);
				CreateEdgeContact(&d_min, &(boxA->hull), &(boxB->hull), &contact_manifold);
			}

			//adjust box velocities for detected collisions
			if(j != index_of_ground)
			{
				ApplyCollisionImpulses(boxA, boxB, contact_manifold.contacts, contact_manifold.num_contacts, 0);
			}
			else
			{
				ApplyCollisionImpulses(boxA, boxB, contact_manifold.contacts, contact_manifold.num_contacts, 1);
				//ApplyCollisionImpulses(&(g_a_box[0]), &(g_a_box[1]), contacts, num_contacts, 1); //1 = is b ground
				//ApplyCollisionImpulses(&(g_a_box[0]), &(g_a_box[1]), contacts, num_contacts, 0); //0 = b is not ground
			}
		}

		//figure out the next combination to check for collision
		//don't want to check a hull against itself or redo checks that have already been done
		j += 1;
		if(j >= num_hulls)
		{
			i += 1;
			j = i + 1;
		}
	}

	//Update actual positions of boxes
	UpdateBoxSimulation(&(g_a_box[0]));
	UpdateBoxSimulation(&(g_a_box[1]));
	
	//printf("box0 linearVel=(%f,%f,%f)\n", g_a_box[0].linearVel[0], g_a_box[0].linearVel[1], g_a_box[0].linearVel[2]);

	g_simulation_step += 1; //let the keyboard handler advance simulation
	printf("step=%d\n", g_simulation_step);
	//printf("box0-pos:%f,%f,%f box0-vel:%f,%f,%f box1-pos:%f,%f,%f box1-vel:%f,%f,%f\n", g_a_box[0].pos[0], g_a_box[0].pos[1], g_a_box[0].pos[2], g_a_box[0].linearVel[0], g_a_box[0].linearVel[1], g_a_box[0].linearVel[2], g_a_box[1].pos[0], g_a_box[1].pos[1], g_a_box[1].pos[2], g_a_box[1].linearVel[0], g_a_box[1].linearVel[1], g_a_box[1].linearVel[2]);
}

static void UpdateBoxSimulation(struct box_struct * pbox)
{

	//CalculateBoxVelocity(pbox, newLinearVel, newAngularMomentum, newAngularVelQ);
	//Idea is for CalculateBoxVelocity() to be called earlier and then this
	//func will update position/orientation

	UpdateBoxVelocity(pbox);

	//update linear position
	pbox->pos[0] += pbox->newLinearVel[0];
	pbox->pos[1] += pbox->newLinearVel[1];
	pbox->pos[2] += pbox->newLinearVel[2];

	//Calculate new orientation
	//q_i+1 = q + (h/2)*w*q		;note: here w is a quaternion q(0,w)
	qMultiply(pbox->newAngularVelQ, pbox->newAngularVelQ, pbox->orientationQ);
	pbox->newAngularVelQ[0] *= 0.5f;
	pbox->newAngularVelQ[1] *= 0.5f;
	pbox->newAngularVelQ[2] *= 0.5f;
	pbox->newAngularVelQ[3] *= 0.5f;
	qAdd(pbox->newAngularVelQ, pbox->orientationQ, pbox->newAngularVelQ);
	//Re-normalize the quaternion
	qNormalize(pbox->newAngularVelQ);

	pbox->orientationQ[0] = pbox->newAngularVelQ[0];
	pbox->orientationQ[1] = pbox->newAngularVelQ[1];
	pbox->orientationQ[2] = pbox->newAngularVelQ[2];
	pbox->orientationQ[3] = pbox->newAngularVelQ[3];
	qConvertToMat3(pbox->newAngularVelQ, pbox->orientation);

	//update the physics hull:
	UpdateHull(&g_base_boxHull, pbox);
}

static void UpdateBoxVelocity(struct box_struct * pbox)
{
	pbox->linearVel[0] = pbox->newLinearVel[0];
	pbox->linearVel[1] = pbox->newLinearVel[1];
	pbox->linearVel[2] = pbox->newLinearVel[2];

	pbox->angularVel[0] = pbox->newAngularVelQ[0];
	pbox->angularVel[1] = pbox->newAngularVelQ[1];
	pbox->angularVel[2] = pbox->newAngularVelQ[2];

	pbox->angularMomentum[0] = pbox->newAngularMomentum[0];
	pbox->angularMomentum[1] = pbox->newAngularMomentum[1];
	pbox->angularMomentum[2] = pbox->newAngularMomentum[2];
}

//This function updates in box_struct:
//	newLinearVel
//	newAngularMomentum
//	newAngularVelQ
static void CalculateNewBoxVelocity(struct box_struct * pbox, 
		float * sumTorques,		//in
		float * sumForces)		//in
{
	float iorient[9];
	float imomentOfInertiaWorld[9];

	//Prepare to calculate angular velocity
	//L_i+1 = L_i + h*T
	vAdd(pbox->newAngularMomentum, pbox->angularMomentum, sumTorques);

	//Transform InverseMomentOfInertia from local to world coordinates
	//I_i^-1 = (R_i)*(I_0^-1)*(R_i^-1)
	memcpy(iorient, pbox->orientation, (9*sizeof(float)));
	mmTranspose3x3(iorient);
	memcpy(imomentOfInertiaWorld, pbox->imomentOfInertia, (9*sizeof(float)));
	mmMultiplyMatrix3x3(imomentOfInertiaWorld, iorient, imomentOfInertiaWorld);
	mmMultiplyMatrix3x3(pbox->orientation, imomentOfInertiaWorld, imomentOfInertiaWorld);

	//Calculate angular velocity for real
	//w_i+1 = (I_i^-1)*(L_i+1)
	pbox->newAngularVelQ[0] = pbox->newAngularMomentum[0];
	pbox->newAngularVelQ[1] = pbox->newAngularMomentum[1];
	pbox->newAngularVelQ[2] = pbox->newAngularMomentum[2];
	mmTransformVec3(imomentOfInertiaWorld, pbox->newAngularVelQ);
	pbox->newAngularVelQ[3] = 0.0f;

	//Calculate new velocity
	pbox->newLinearVel[0] = pbox->linearVel[0] + (sumForces[0]/pbox->mass);
	pbox->newLinearVel[1] = pbox->linearVel[1] + (sumForces[1]/pbox->mass);
	pbox->newLinearVel[2] = pbox->linearVel[2] + (sumForces[2]/pbox->mass);	
}

static void ApplyCollisionImpulses(struct box_struct * boxA, struct box_struct * boxB, struct contact_info_struct * contacts, int num_contacts, int is_b_ground)
{
	//TODO: Need to find out if this is bad Assume: Max of 4 contacts. This might be dumb?
	float impulse_k[4];
	float impulse[4];
	float old_impulse[4];
	float cur_impulse_mag;
	float impulse_vec[3];
	float delta_linear_vel[3];
	float * r[2]; //0 = boxA r, 1 = box B r. where r is vec from box CG to contact point.
	float r_boxA[3] = {0.0f, 0.0f, 0.0f};
	float r_boxB[3] = {0.0f, 0.0f, 0.0f};
	float temp_vec[3];
	float cross_vec[3];
	float sum_vec[3];
	float vel_bias = 0.0f;
	float sumTorques[6];	//2 vec3s
	float sumForces[6];		//2 vec3s
	int i;
	int j;
	int k;

	memset(impulse, 0, 4*sizeof(float));
	memset(old_impulse, 0, 4*sizeof(float));

	//calculate k_constant for Impulse from contact normal. This is constant throughout iterations.
	r[0] = r_boxA;
	r[1] = r_boxB;
	for(i = 0; i < num_contacts; i++)
	{
		//k_n = (1/m_1) + (1/m_2) + (( (I_1^-1*(r1 cross n) cross r1) + I_2^-1(r2 cross n) cross r2) dot n)

		//I_1^-1*(r1 cross n) cross r1
		vSubtract(r[0], contacts[i].point, boxA->pos);
		vCrossProduct(cross_vec, r[0], contacts[i].normal);
		vCrossProduct(temp_vec, cross_vec, r[0]);
		mmTransformVec3(boxA->imomentOfInertia, temp_vec); //TODO: Need to fix this. box->imomentOfInertia is in model space, but needs to be in world space.
		sum_vec[0] = temp_vec[0];	//sum_vec will hold the sum of the two terms with I_1^-1 and I_2^-1
		sum_vec[1] = temp_vec[1];
		sum_vec[2] = temp_vec[2];
		
		//I_2^-1*(r2 cross n) cross r2
		if(is_b_ground == 0)
		{
			vSubtract(r[1], contacts[i].point, boxB->pos);
			vCrossProduct(cross_vec, r[1], contacts[i].normal);
			vCrossProduct(temp_vec, cross_vec, r[1]);
			mmTransformVec3(boxB->imomentOfInertia, temp_vec);
			sum_vec[0] += temp_vec[0];
			sum_vec[1] += temp_vec[1];
			sum_vec[2] += temp_vec[2];
		}

		impulse_k[i] = (1/boxA->mass) + vDotProduct(sum_vec, contacts[i].normal);

		//If b is ground then skip calculating its mass
		if(is_b_ground == 0)
		{
			impulse_k[i] += (1/boxB->mass);
		}
	}

	//Iterate over the impulses at least 10 times
	printf("box0 init linearVel=(%f,%f,%f) angularVel=(%f,%f,%f)\n", boxA->linearVel[0], boxA->linearVel[1], boxA->linearVel[2], boxA->angularVel[0], boxA->angularVel[1], boxA->angularVel[2]);
	for(k = 0; k < 10; k++)
	{
		//printf("impulse round %d:\n", k);
		memset(sumForces, 0, 6*sizeof(float));
		memset(sumTorques, 0, 6*sizeof(float));
		for(j = 0; j < num_contacts; j++)
		{
			//dV = v_2 + (w_2 cross r_2) - v_1 - (w_1 cross r_1)
			//TODO: Shouldn't we be using newLinearVel here? otherwise how does linearVel
			//get updated?
			vSubtract(r[0], contacts[j].point, boxA->pos);
			vCrossProduct(cross_vec, boxA->angularVel, r[0]);
			vAdd(delta_linear_vel, boxA->linearVel, cross_vec);
			vSubtract(r[1], contacts[j].point, boxB->pos);
			vCrossProduct(cross_vec, boxB->angularVel, r[1]);
			vSubtract(delta_linear_vel, delta_linear_vel, boxB->linearVel);
			vSubtract(delta_linear_vel, delta_linear_vel, cross_vec);

			//same calls as above just using newLinearVel, this doesn't work right.
			/*vCrossProduct(cross_vec, boxA->newAngularVelQ, r[0]);
			vAdd(delta_linear_vel, boxA->newLinearVel, cross_vec);
			vCrossProduct(cross_vec, boxB->newAngularVelQ, r[1]);
			vSubtract(delta_linear_vel, delta_linear_vel, boxB->newLinearVel);
			vSubtract(delta_linear_vel, delta_linear_vel, cross_vec);*/

			//max[ (-dV dot n + v_bias)/k_n , 0]
			vel_bias = CalcBaumgarteBias(contacts[j].penetration);
			cur_impulse_mag = (((-1.0f*vDotProduct(delta_linear_vel, contacts[j].normal)) + vel_bias)/impulse_k[j]);
	
			//clamp the accumulated impulse
			old_impulse[j] = impulse[j];
			impulse[j] += cur_impulse_mag;
			if(impulse[j] < 0.0f)	//clamp 0
				impulse[j] = 0.0f;

			//apply impulse for box A
			impulse_vec[0] = contacts[j].normal[0];
			impulse_vec[1] = contacts[j].normal[1];
			impulse_vec[2] = contacts[j].normal[2];
			impulse_vec[0] *= (impulse[j] - old_impulse[j]);
			impulse_vec[1] *= (impulse[j] - old_impulse[j]);
			impulse_vec[2] *= (impulse[j] - old_impulse[j]);
			vAdd(sumForces, sumForces, impulse_vec);

			vSubtract(temp_vec, contacts[j].point, boxA->pos); 	//calculate R
			vCrossProduct(cross_vec, temp_vec, impulse_vec);	//calculate torque
			vAdd(sumTorques, sumTorques, cross_vec);

			printf("\tcontact=%d impulse=%f contactpos=(%f,%f,%f)\n", j, (impulse[j]-old_impulse[j]), contacts[j].point[0], contacts[j].point[1], contacts[j].point[2]);

			//TODO: Remove this but try to recalc velocity after every impulse
			CalculateNewBoxVelocity(boxA, cross_vec, impulse_vec);
			UpdateBoxVelocity(boxA);

			//apply impulse for box B
			if(is_b_ground == 0) //only add force if B is a regular object. If B is ground treat it as infinite mass.
			{
				impulse_vec[0] = -1.0f*contacts[j].normal[0];
				impulse_vec[1] = -1.0f*contacts[j].normal[1];
				impulse_vec[2] = -1.0f*contacts[j].normal[2];
				impulse_vec[0] *= (impulse[j] - old_impulse[j]);
				impulse_vec[1] *= (impulse[j] - old_impulse[j]);
				impulse_vec[2] *= (impulse[j] - old_impulse[j]);
				vAdd((sumForces+3), (sumForces+3), impulse_vec);

				vSubtract(temp_vec, contacts[j].point, boxB->pos);
				vCrossProduct(cross_vec, temp_vec, impulse_vec);
				vAdd((sumTorques+3), (sumTorques+3), cross_vec);

				CalculateNewBoxVelocity(boxB, cross_vec, impulse_vec);
				UpdateBoxVelocity(boxB);
			}
		}
	
		//now calculate new linear and angular velocities
		//CalculateNewBoxVelocity(boxA, sumTorques, sumForces);
		//UpdateBoxVelocity(boxA);
		//if(is_b_ground == 0)
		//{
		//	CalculateNewBoxVelocity(boxB, (sumTorques+3), (sumForces+3));
		//	UpdateBoxVelocity(boxB);
		//}
		printf("\tboxA endLinearVel=(%f,%f,%f) endAngularVel=(%f,%f,%f)\n", boxA->newLinearVel[0], boxA->newLinearVel[1], boxA->newLinearVel[2], boxA->newAngularVelQ[0], boxA->newAngularVelQ[1], boxA->newAngularVelQ[2]);
	}

	//print out the final impulses. impulse[j] should have the sum of all impulses from each iteration.
	printf("final impulses: ");
	for(i = 0; i < num_contacts; i++)
	{
		printf("%f ", impulse[i]);
	}
	printf("\n");

	//CalculateBoxVelocity() now has updated newLinearVel, newAngularMomentum, newAngularVelQ
	//ready to apply for position
}

static float CalcBaumgarteBias(float penetration)
{
	float k_bias_factor = 0.01f;		//configurable
	float k_bias_margin = 0.0001f;	//configurable
	float dt = 0.016666666f;		//1/60 Hz. timestep size.
	float bias;
	float dist;

	//v_bias = k_bias_factor/dt * max(0, penetration - k_bias_margin)
	penetration = fabs(penetration);
	dist = penetration - k_bias_margin;
	if(dist < 0.0f)
		dist = 0.0f;
	bias = (k_bias_factor*dist)/dt;

	return bias;
}

/*
This function is here because the values associated with angular displacement are all 3x3 matrices, but
the matrices that actually transform the model are 4x4 matrices.
*/
void VehicleConvertDisplacementMat3To4(float * mat3, float * mat4)
{
	mat4[0] = mat3[0];
	mat4[1] = mat3[1];
	mat4[2] = mat3[2];
	mat4[3] = 0.0f;
	
	mat4[4] = mat3[3];
	mat4[5] = mat3[4];
	mat4[6] = mat3[5];
	mat4[7] = 0.0f;
	
	mat4[8] = mat3[6];
	mat4[9] = mat3[7];
	mat4[10] = mat3[8];
	mat4[11] = 0.0f;
	
	mat4[12] = 0.0f;
	mat4[13] = 0.0f;
	mat4[14] = 0.0f;
	mat4[15] = 1.0f;
}

/*
Calculates the elapsed time between end and start and returns result.
*/
void GetElapsedTime(struct timespec * start, struct timespec * end, struct timespec * result)
{
	time_t * p_larger_sec;
	time_t * p_smaller_sec;
	long * p_larger_nsec;
	long * p_smaller_nsec;
	
	//determine rollover
	if(start->tv_sec > end->tv_sec)
	{
		result->tv_sec = (0x7FFFFFFFFFFFFFFF - start->tv_sec) + end->tv_sec;//assume time_t is long because i'm an idiot
	}
	else
	{
		result->tv_sec = end->tv_sec - start->tv_sec;
	}
	
	//determine rollover
	if(end->tv_nsec < start->tv_nsec)
	{
		result->tv_sec -= 1; //carry the seconds
		result->tv_nsec = (1000000000 - start->tv_nsec) + end->tv_nsec;
	}
	else
	{
		result->tv_nsec = end->tv_nsec - start->tv_nsec;
	}
}

static int InitHull(float * positions, int num_positions, struct box_collision_struct * phull)
{
	//Prolly should make this automated
	memset(phull, 0, sizeof(struct box_collision_struct));

	phull->num_pos = num_positions;
	phull->positions = (float*)malloc(num_positions*3*sizeof(float));
	if(phull->positions == 0)
		return 0;
	memcpy(phull->positions, positions, num_positions*3*sizeof(float));

	phull->num_faces = 6;
	phull->faces = (struct face_struct*)malloc(6*sizeof(struct face_struct));
	if(phull->faces == 0)
		return 0;

	//+x face
	phull->faces[0].num_verts = 4;
	phull->faces[0].normal[0] = 1.0f;
	phull->faces[0].normal[1] = 0.0f;
	phull->faces[0].normal[2] = 0.0f;
	phull->faces[0].i_vertices[0] = 0;
	phull->faces[0].i_vertices[1] = 4;
	phull->faces[0].i_vertices[2] = 5;
	phull->faces[0].i_vertices[3] = 1;

	//+z face
	phull->faces[1].num_verts = 4;
	phull->faces[1].normal[0] = 0.0f;
	phull->faces[1].normal[1] = 0.0f;
	phull->faces[1].normal[2] = 1.0f;
	phull->faces[1].i_vertices[0] = 1;
	phull->faces[1].i_vertices[1] = 5;
	phull->faces[1].i_vertices[2] = 6;
	phull->faces[1].i_vertices[3] = 2;

	//-x face
	phull->faces[2].num_verts = 4;
	phull->faces[2].normal[0] = -1.0f;
	phull->faces[2].normal[1] = 0.0f;
	phull->faces[2].normal[2] = 0.0f;
	phull->faces[2].i_vertices[0] = 2;
	phull->faces[2].i_vertices[1] = 6;
	phull->faces[2].i_vertices[2] = 7;
	phull->faces[2].i_vertices[3] = 3;
	
	//-z face
	phull->faces[3].num_verts = 4;
	phull->faces[3].normal[0] = 0.0f;
	phull->faces[3].normal[1] = 0.0f;
	phull->faces[3].normal[2] = -1.0f;
	phull->faces[3].i_vertices[0] = 3;
	phull->faces[3].i_vertices[1] = 0;
	phull->faces[3].i_vertices[2] = 4;
	phull->faces[3].i_vertices[3] = 7;

	//+y face
	phull->faces[4].num_verts = 4;
	phull->faces[4].normal[0] = 0.0f;
	phull->faces[4].normal[1] = 1.0f;
	phull->faces[4].normal[2] = 0.0f;
	phull->faces[4].i_vertices[0] = 4;
	phull->faces[4].i_vertices[1] = 7;
	phull->faces[4].i_vertices[2] = 6;
	phull->faces[4].i_vertices[3] = 5;

	//-y face
	phull->faces[5].num_verts = 4;
	phull->faces[5].normal[0] = 0.0f;
	phull->faces[5].normal[1] = -1.0f;
	phull->faces[5].normal[2] = 0.0f;
	phull->faces[5].i_vertices[0] = 1;
	phull->faces[5].i_vertices[1] = 2;
	phull->faces[5].i_vertices[2] = 3;
	phull->faces[5].i_vertices[3] = 0;

	phull->num_edges = 12;
	phull->edges = (struct edge_struct*)malloc(12*sizeof(struct edge_struct));
	if(phull->edges == 0)
		return 0;

	//0 - 1
	//phull->edges[0].normal[0] = 0.0f;
	//phull->edges[0].normal[1] = 0.0f;
	//phull->edges[0].normal[2] = 1.0f;
	phull->edges[0].i_vertices[0] = 0;
	phull->edges[0].i_vertices[1] = 1;
	phull->edges[0].i_face[0] = 0;
	phull->edges[0].i_face[1] = 5;
	vCrossProduct(phull->edges[0].normal, phull->faces[0].normal, phull->faces[5].normal);

	//1 - 2
	//phull->edges[1].normal[0] = -1.0f;
	//phull->edges[1].normal[1] = 0.0f;
	//phull->edges[1].normal[2] = 0.0f;
	phull->edges[1].i_vertices[0] = 1;
	phull->edges[1].i_vertices[1] = 2;
	phull->edges[1].i_face[0] = 1;
	phull->edges[1].i_face[1] = 5;
	vCrossProduct(phull->edges[1].normal, phull->faces[1].normal, phull->faces[5].normal);

	//2 - 3
	//phull->edges[2].normal[0] = 0.0f;
	//phull->edges[2].normal[1] = 0.0f;
	//phull->edges[2].normal[2] = -1.0f;
	phull->edges[2].i_vertices[0] = 2;
	phull->edges[2].i_vertices[1] = 3;
	phull->edges[2].i_face[0] = 2;
	phull->edges[2].i_face[1] = 5;
	vCrossProduct(phull->edges[2].normal, phull->faces[2].normal, phull->faces[5].normal);

	//3 - 0
	//phull->edges[3].normal[0] = 1.0f;
	//phull->edges[3].normal[1] = 0.0f;
	//phull->edges[3].normal[2] = 0.0f;
	phull->edges[3].i_vertices[0] = 3;
	phull->edges[3].i_vertices[1] = 0;
	phull->edges[3].i_face[0] = 3;
	phull->edges[3].i_face[1] = 5;
	vCrossProduct(phull->edges[3].normal, phull->faces[3].normal, phull->faces[5].normal);

	//4 - 5
	//phull->edges[4].normal[0] = 0.0f;
	//phull->edges[4].normal[1] = 0.0f;
	//phull->edges[4].normal[2] = -1.0f;
	phull->edges[4].i_vertices[0] = 4;
	phull->edges[4].i_vertices[1] = 5;
	phull->edges[4].i_face[0] = 4;
	phull->edges[4].i_face[1] = 0;
	vCrossProduct(phull->edges[4].normal, phull->faces[4].normal, phull->faces[0].normal);

	//5 - 6
	//phull->edges[5].normal[0] = -1.0f;
	//phull->edges[5].normal[1] = 0.0f;
	//phull->edges[5].normal[2] = 0.0f;
	phull->edges[5].i_vertices[0] = 5;
	phull->edges[5].i_vertices[1] = 6;
	phull->edges[5].i_face[0] = 4;
	phull->edges[5].i_face[1] = 1;
	vCrossProduct(phull->edges[5].normal, phull->faces[4].normal, phull->faces[1].normal);

	//6 - 7
	//phull->edges[6].normal[0] = 0.0f;
	//phull->edges[6].normal[1] = 0.0f;
	//phull->edges[6].normal[2] = -1.0f;
	phull->edges[6].i_vertices[0] = 6;
	phull->edges[6].i_vertices[1] = 7;
	phull->edges[6].i_face[0] = 4;
	phull->edges[6].i_face[1] = 2;
	vCrossProduct(phull->edges[6].normal, phull->faces[4].normal, phull->faces[2].normal);

	//7 - 4
	//phull->edges[7].normal[0] = 1.0f;
	//phull->edges[7].normal[1] = 0.0f;
	//phull->edges[7].normal[2] = 0.0f;
	phull->edges[7].i_vertices[0] = 7;
	phull->edges[7].i_vertices[1] = 4;
	phull->edges[7].i_face[0] = 4;
	phull->edges[7].i_face[1] = 3;
	vCrossProduct(phull->edges[7].normal, phull->faces[4].normal, phull->faces[3].normal);

	//0 - 4
	//phull->edges[8].normal[0] = 0.0f;
	//phull->edges[8].normal[1] = 1.0f;
	//phull->edges[8].normal[2] = 0.0f;
	phull->edges[8].i_vertices[0] = 0;
	phull->edges[8].i_vertices[1] = 4;
	phull->edges[8].i_face[0] = 0;
	phull->edges[8].i_face[1] = 3;
	vCrossProduct(phull->edges[8].normal, phull->faces[0].normal, phull->faces[3].normal);

	//1 - 5
	//phull->edges[9].normal[0] = 0.0f;
	//phull->edges[9].normal[1] = 1.0f;
	//phull->edges[9].normal[2] = 0.0f;
	phull->edges[9].i_vertices[0] = 1;
	phull->edges[9].i_vertices[1] = 5;
	phull->edges[9].i_face[0] = 0;
	phull->edges[9].i_face[1] = 1;
	vCrossProduct(phull->edges[9].normal, phull->faces[0].normal, phull->faces[1].normal);

	//2 - 6
	//phull->edges[10].normal[0] = 0.0f;
	//phull->edges[10].normal[1] = 1.0f;
	//phull->edges[10].normal[2] = 0.0f;
	phull->edges[10].i_vertices[0] = 2;
	phull->edges[10].i_vertices[1] = 6;
	phull->edges[10].i_face[0] = 1;
	phull->edges[10].i_face[1] = 2;
	vCrossProduct(phull->edges[10].normal, phull->faces[1].normal, phull->faces[2].normal);

	//3 - 7
	//phull->edges[0].normal[0] = 0.0f;
	//phull->edges[0].normal[1] = 1.0f;
	//phull->edges[0].normal[2] = 0.0f;
	phull->edges[11].i_vertices[0] = 3;
	phull->edges[11].i_vertices[1] = 7;
	phull->edges[11].i_face[0] = 2;
	phull->edges[11].i_face[1] = 3;
	vCrossProduct(phull->edges[11].normal, phull->faces[2].normal, phull->faces[3].normal);

	return 1;
}

static int InitPlaneHull(float * positions, int num_positions, struct box_collision_struct * phull)
{
	memset(phull, 0, sizeof(struct box_collision_struct));

	phull->positions = (float*)malloc(num_positions*3*sizeof(float));
	if(phull->positions == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}
	phull->num_pos =  num_positions;
	memcpy(phull->positions, positions, 3*num_positions*sizeof(float));

	//faces
	phull->num_faces = 2;
	phull->faces = (struct face_struct*)malloc(2*sizeof(struct face_struct));
	if(phull->faces == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}

	phull->faces[0].normal[0] = 0.0f;
	phull->faces[0].normal[1] = 1.0f;
	phull->faces[0].normal[2] = 0.0f;
	phull->faces[0].num_verts = 4;
	phull->faces[0].i_vertices[0] = 0;
	phull->faces[0].i_vertices[1] = 1;
	phull->faces[0].i_vertices[2] = 2;
	phull->faces[0].i_vertices[3] = 3;

	phull->faces[1].normal[0] = 0.0f;
	phull->faces[1].normal[1] = -1.0f;
	phull->faces[1].normal[2] = 0.0f;
	phull->faces[1].num_verts = 4;
	phull->faces[1].i_vertices[0] = 3;
	phull->faces[1].i_vertices[1] = 2;
	phull->faces[1].i_vertices[2] = 1;
	phull->faces[1].i_vertices[3] = 0;

	//edges
	phull->num_edges = 4;
	phull->edges = (struct edge_struct*)malloc(4*sizeof(struct edge_struct));
	if(phull->edges == 0)
	{
		printf("%s: error line %d\n", __func__, __LINE__);
		return 0;
	}

	//0 - 1
	phull->edges[0].i_vertices[0] = 0;
	phull->edges[0].i_vertices[1] = 1;
	phull->edges[0].i_face[0] = 1;
	phull->edges[0].i_face[1] = 0;
	phull->edges[0].normal[0] = -1.0f;
	phull->edges[0].normal[1] = 0.0f;
	phull->edges[0].normal[2] = 0.0f;

	//1 - 2
	phull->edges[1].i_vertices[0] = 1;
	phull->edges[1].i_vertices[1] = 2;
	phull->edges[1].i_face[0] = 1;
	phull->edges[1].i_face[1] = 0;
	phull->edges[1].normal[0] = 0.0f;
	phull->edges[1].normal[1] = 0.0f;
	phull->edges[1].normal[2] = 1.0f;

	//2 - 3
	phull->edges[2].i_vertices[0] = 2;
	phull->edges[2].i_vertices[1] = 3;
	phull->edges[2].i_face[0] = 1;
	phull->edges[2].i_face[1] = 0;
	phull->edges[2].normal[0] = 1.0f;
	phull->edges[2].normal[1] = 0.0f;
	phull->edges[2].normal[2] = 0.0f;

	//3 - 0
	phull->edges[3].i_vertices[0] = 3;
	phull->edges[3].i_vertices[1] = 0;
	phull->edges[3].i_face[0] = 1;
	phull->edges[3].i_face[1] = 0;
	phull->edges[0].normal[0] = 0.0f;
	phull->edges[0].normal[1] = 0.0f;
	phull->edges[0].normal[2] = -1.0f;

	return 1;
}

//Assume: UpdateSimulation updated the location of the hull in
//world space.
//struct d_min_struct d_min; //I made a struct for this because I need a way of saying in the first iteration that d_min,s_min aren't initialized.
int FindSeparatingAxis(struct box_struct * boxA, struct box_struct * boxB, struct d_min_struct * d_min)
{
	struct box_collision_struct tempHull[2];
	struct box_collision_struct * hullA;
	struct box_collision_struct * hullB;
	float normal[3];
	float temp_vec[3];
	float normalB[3];
	float * point_on_plane;
	float d;
	float ffaceWeightBias = 0.001f; //this is a factor to prefer face d_min selection, over which a edge d_min needs to be better than a face d_min.
	int i;
	int i_edge;
	int j_edge;
	int debug_num_edgechecks_skipped = 0;
	int r;

	//Assume hulls have been transformed to world-coordinates
	hullA = &(boxA->hull);
	hullB = &(boxB->hull);

	memset(d_min, 0, sizeof(struct d_min_struct));

	//Now check edges.
	//Do edge check first so, the face checks override the edges if they have the same d_min
	d_min->cur_check = 1; //set that we are checking edges
	for(i_edge = 0; i_edge < hullA->num_edges; i_edge++)
	{
		for(j_edge = 0; j_edge < hullB->num_edges; j_edge++)
		{
			vCrossProduct(normal, hullA->edges[i_edge].normal, hullB->edges[j_edge].normal);
			
			//magnitude will be 0 when edges are parallel. skip this situation.
			//if(vMagnitude(normal) != 0)
			if(vMagnitude(normal) > 0.001f)
			{
				normalB[0] = hullB->edges[j_edge].normal[0];
				normalB[1] = hullB->edges[j_edge].normal[1];
				normalB[1] = hullB->edges[j_edge].normal[2];
				r = FilterEdgeCheck(hullA, hullB, i_edge, j_edge, normalB);	
				if(r == 0) 
				{
					debug_num_edgechecks_skipped += 1;
					continue; //if edges aren't supporting features skip the check
				}
				vCrossProduct(normal, hullA->edges[i_edge].normal, normalB); //recalculate normal incase normalB got flipped

				point_on_plane = hullA->positions+((hullA->edges[i_edge].i_vertices[0])*3);

				//make sure that s points towards box A's origin to keep consistent with how
				//s is defined.
				vSubtract(temp_vec, boxA->pos, point_on_plane);
				d = vDotProduct(normal, temp_vec);
				if(d < 0.0f)
				{
					normal[0] *= -1.0f;
					normal[1] *= -1.0f;
					normal[2] *= -1.0f;
				}

				vNormalize(normal);
			
				//r = SATCheckDirection(normal, point_on_plane, hullA, hullB, d_min);
				r = CheckEdgePlane(boxA, boxB, normal, point_on_plane, d_min);
				if(r == 1)
				{
					d_min->i_face = -1;
					d_min->i_edge[0] = i_edge;
					d_min->i_edge[1] = j_edge;
					//since this is edge don't set i_hull.
				}

				/*normal[0] *= -1.0f;
				normal[1] *= -1.0f;
				normal[2] *= -1.0f;
				r = SATCheckDirection(normal, point_on_plane, hullA, hullB, d_min);
				if(r == 1)
				{
					d_min->i_face = -1;
					d_min->i_edge[0] = i_edge;
					d_min->i_edge[1] = j_edge;
					//since this is edge don't set i_hull
				}*/
			}
		}
	}
	//we want to save the d_min and s_min and associate it with edges, This is so we can choose
	//what kind of contact we need to make.
	d_min->s_min_edges[0] = d_min->s_min[0];
	d_min->s_min_edges[1] = d_min->s_min[1];
	d_min->s_min_edges[2] = d_min->s_min[2];
	d_min->d_min_edges = d_min->d_min;

	d_min->is_initialized = 0; //reset d_min struct
	//First check s & d for faces in hull A
	for(i = 0; i < hullA->num_faces; i++)
	{
		//calculate s vec
		//TODO: The book's treatment of s to use when checking faces seems problematic, but the pdf seems ok.
		//book says to negate the face-normals from hullA ... but why?
		//convection is for s to point into hullA, this is the direction that the impulse would be applied to A for any collision
		//contact. 
		memcpy(normal, hullA->faces[i].normal, 3*sizeof(float));
		normal[0] *= -1.0f;
		normal[1] *= -1.0f;
		normal[2] *= -1.0f;

		//get the point on plane
		point_on_plane = hullA->positions+((hullA->faces[i].i_vertices[0])*3);

		//printf("FindSeparatingAxis: check hullA i_face=%d\n", i);
		d_min->cur_check = 0; //indicate to SATCheckDirection() that we are using faces to get s_min
		r = SATCheckDirection(normal, point_on_plane, hullA, hullB, d_min); 
		if(r == 1) //if SATCheckDirection found a new min update the information
		{
			d_min->i_face = i;
			d_min->i_hull[0] = 0; //A
			d_min->i_hull[1] = 1; //B
		}
	}

	//Now check s & d for faces in hull B
	for(i = 0; i < hullB->num_faces; i++)
	{
		memcpy(normal, hullB->faces[i].normal, 3*sizeof(float)); //copy to normal, so that for debug I can invert the normal vector.
		//normal[0] *= -1.0f;
		//normal[1] *= -1.0f;
		//normal[2] *= -1.0f;

		//get point on plane
		point_on_plane = hullB->positions+((hullB->faces[i].i_vertices[0])*3);

		//printf("FindSeparatingAxis: check hullB i_face=%d\n", i);
		d_min->cur_check = 0; //indicate to SATCheckDirection() that we are using faces to get s_min
		r = SATCheckDirection(normal, point_on_plane, hullA, hullB, d_min);
		if(r == 1)
		{
			d_min->i_face = i;
			d_min->i_hull[0] = 1; //B
			d_min->i_hull[1] = 0; //A
		}
	}
	d_min->s_min_faces[0] = d_min->s_min[0];
	d_min->s_min_faces[1] = d_min->s_min[1];
	d_min->s_min_faces[2] = d_min->s_min[2];
	d_min->d_min_faces = d_min->d_min;

	//TODO: Remove this debug
	if(d_min->d_min_faces <= 0.0f)
	{
		printf("faces has d_min=%f\n", d_min->d_min_faces);
	}

	//now select the final d_min between faces and edges
	//pick face if it has a smaller penetration than edge
	//an edge d_min has to be better by a FFACEWEIGHTBIAS to be chosen
	/*if((d_min->d_min_faces+ffaceWeightBias) >= d_min->d_min_edges)
	{
		d_min->d_min = d_min->d_min_faces;
		d_min->s_min[0] = d_min->s_min_faces[0];
		d_min->s_min[1] = d_min->s_min_faces[1];
		d_min->s_min[2] = d_min->s_min_faces[2];
		d_min->source = 0;
	}
	else
	{
		printf("picking edge.d_min=%f over face.d_min=%f\n", d_min->d_min_edges, d_min->d_min_faces);
		d_min->d_min = d_min->d_min_edges;
		d_min->s_min[0] = d_min->s_min_edges[0];
		d_min->s_min[1] = d_min->s_min_edges[1];
		d_min->s_min[2] = d_min->s_min_edges[2];
		d_min->source = 1;
	}*/
	if(d_min->d_min_faces <= 0.0f && d_min->d_min_edges <= 0.0f)
	{
		if((d_min->d_min_faces+ffaceWeightBias) >= d_min->d_min_edges)
		{
			d_min->d_min = d_min->d_min_faces;
			d_min->s_min[0] = d_min->s_min_faces[0];
			d_min->s_min[1] = d_min->s_min_faces[1];
			d_min->s_min[2] = d_min->s_min_faces[2];
			d_min->source = 0;
		}
		else
		{
			d_min->d_min = d_min->d_min_edges;
			d_min->s_min[0] = d_min->s_min_edges[0];
			d_min->s_min[1] = d_min->s_min_edges[1];
			d_min->s_min[2] = d_min->s_min_edges[2];
			d_min->source = 1;
		}
		r = 0; //no separating axis. found overlap.
	}
	else if(d_min->d_min_faces <= 0.0f)
	{
		d_min->d_min = d_min->d_min_faces;
		d_min->s_min[0] = d_min->s_min_faces[0];
		d_min->s_min[1] = d_min->s_min_faces[1];
		d_min->s_min[2] = d_min->s_min_faces[2];
		d_min->source = 0;
		r = 0; //no separating axis. found overlap.
	}
	else if(d_min->d_min_edges <= 0.0f)
	{
		d_min->d_min = d_min->d_min_edges;
		d_min->s_min[0] = d_min->s_min_edges[0];
		d_min->s_min[1] = d_min->s_min_edges[1];
		d_min->s_min[2] = d_min->s_min_edges[2];
		d_min->source = 1;
		r = 0; //no separating axis. found overlap.
	}
	else
	{
		r = 1; //found separating axis.
	}

	//if(d_min->d_min <= 0.0f)
	//	printf("separating axis: %d edge checks skipped.\n", debug_num_edgechecks_skipped);

	printf("separating axis: r=%d\n", r);
	printf("separating axis: d_min=%f source=%d\n", d_min->d_min, d_min->source);
	if(d_min->source == 0)
	{
		printf("i_face=%d i_hull_ref=%d i_hull_inc=%d\n", d_min->i_face, d_min->i_hull[0], d_min->i_hull[1]);
	}
	if(d_min->source == 1)
	{
		printf("i_edge_0=%d i_edge_1=%d\n", d_min->i_edge[0], d_min->i_edge[1]);
	}
	printf("s_min: (%f,%f,%f)\n", d_min->s_min[0], d_min->s_min[1], d_min->s_min[2]);
	
	return r;
}

//'point_on_plane': This vec3 pos that is on the plane with normal s_vec3.
//returns 1 if updated d_min_struct
static int SATCheckDirection(float * s_vec3, float * point_on_plane, struct box_collision_struct * hullA, struct box_collision_struct * hullB, struct d_min_struct * d_min)
{
	float d[2];
	float d_sum;
	float normal3[3];
	int r=0;

	//check vertices of hull A
	d[0] = SATFindSupport(hullA, s_vec3, point_on_plane);

	//check vertices of hull B, so need to negate s
	memcpy(normal3, s_vec3, 3*sizeof(float));
	normal3[0] *= -1.0f;
	normal3[1] *= -1.0f;
	normal3[2] *= -1.0f;
	d[1] = SATFindSupport(hullB, normal3, point_on_plane);
	d_sum = d[0] + d[1];
	//printf("SATCheckDirection: dfinal=%f check HullA s=(%f,%f,%f) d=%f, check HullB s=(%f,%f,%f) d=%f\n", d_sum, s_vec3[0], s_vec3[1], s_vec3[2], d[0], normal3[0], normal3[1], normal3[2], d[1]);

	//if this is the first iteration just set d_min,s_min
	//for all other iterations perform the check
	if(d_min->is_initialized == 0)
	{
		d_min->d_min = d_sum;
		d_min->s_min[0] = s_vec3[0];
		d_min->s_min[1] = s_vec3[1];
		d_min->s_min[2] = s_vec3[2];
		r = 1;
		d_min->is_initialized = 1;
	}
	else
	{
		if(d_sum > d_min->d_min) 
		{
			d_min->d_min = d_sum;
			d_min->s_min[0] = s_vec3[0];
			d_min->s_min[1] = s_vec3[1];
			d_min->s_min[2] = s_vec3[2];
			r = 1;
		}
	}

	return r;
}

//'point_on_plane': This is vec3 pos that is on the plane with normal s_vec3. This I think might be needed
//to properly calculate the dot product which gives a distance
//note: point_on_plane is NOT used currently
static float SATFindSupport(struct box_collision_struct * hull, float * s_vec3, float * point_on_plane)
{
	int i;
	int min_i;
	float temp_vec[3];
	float min_dot;
	float dot;

	//look for the vertex on the hull that has the greatest projection, since s points into the shape, the
	//largest projection will be the largest negative
	for(i = 0; i < hull->num_pos; i++)
	{
		//vSubtract(temp_vec, (hull->positions+(i*3)), point_on_plane);
		memcpy(temp_vec, (hull->positions+(i*3)), 3*sizeof(float));
		dot = vDotProduct(s_vec3, temp_vec);

		if(i == 0) //need to jumpstart min_dot for the first iteration, since we are looking for a minimum
		{
			min_dot = dot;
			min_i = i;
		}
		else
		{
			if(dot < min_dot)	//by convention s points inwards (opposite of a face normal) so < would give us the vertex the most towards the other hull.
			{
				min_i = i;
				min_dot = dot;
			}
		}
	}

	return min_dot;
}

//This func returns 1 if conditions for edges to be a support feature are met:
//  both conditions must be satisfied:
//	1. sign( eA <dot> nB_0) != sign( eA <dot> nB_1)
//	2. sign( eB <dot> nA_0) != sign( eB <dot> nA_1)
static int FilterEdgeCheck(struct box_collision_struct * hullA, struct box_collision_struct * hullB, int i_edgeA, int j_edgeB, float * normalB)
{
	struct edge_struct * edge[2];
	float cross_vec[3];
	float mag;
	float dot;
	int edge_B_special_case=0;
	int sign_eA_nB0; //1 = positive, 0 = negative
	int sign_eA_nB1;
	int sign_eB_nA0;
	int sign_eB_nA1;
	int r=1;

	edge[0] = hullA->edges+i_edgeA;
	edge[1] = hullB->edges+j_edgeB;

	//check for bad normalB
	if((edge[1]->normal[0] != normalB[0]) && (edge[1]->normal[1] != normalB[1]) && (edge[1]->normal[2] != normalB[2]))
	{
		printf("***FilterEdgeCheck: ***NORMALB DOES NOT MATCH EDGEB-NORMAL***\n");
		return 1;
	}

	// sign( eA <dot> nB0 ) != sign( eA <dot> nB1)
	dot = vDotProduct(edge[0]->normal, hullB->faces[(edge[1]->i_face[0])].normal);
	if(dot < 0.0f)
	{
		sign_eA_nB0 = 0;
	}
	else
	{
		sign_eA_nB0 = 1;
	}
	dot = vDotProduct(edge[0]->normal, hullB->faces[(edge[1]->i_face[1])].normal);
	if(dot < 0.0f)
	{
		sign_eA_nB1 = 0;
	}
	else
	{
		sign_eA_nB1 = 1;
	}

	// sign( eB <dot> nA0 ) != sign( eB <dot> nA1)
	dot = vDotProduct(edge[1]->normal, hullA->faces[(edge[0]->i_face[0])].normal);
	if(dot < 0.0f)
	{
		sign_eB_nA0 = 0;
	}
	else
	{
		sign_eB_nA0 = 1;
	}
	dot = vDotProduct(edge[1]->normal, hullA->faces[(edge[0]->i_face[1])].normal);
	if(dot < 0.0f)
	{
		sign_eB_nA1 = 0;
	}
	else
	{
		sign_eB_nA1 = 1;
	}

	//check for special case of 180 degree separation of edge boundary face normals
	vCrossProduct(cross_vec, hullB->faces[(edge[1]->i_face[0])].normal, hullB->faces[(edge[1]->i_face[1])].normal);
	mag = vMagnitude(cross_vec);
	if(mag == 0.0f)
	{
		//special case. edgeB can be negated to pass this check.
		//only proceed if edgeA part of check passes
		if(sign_eA_nB0 == sign_eA_nB1)
		{
			r = 0; //skip, signs need to be different
		}
		else
		{
			if(sign_eB_nA0 != sign_eB_nA1)
			{
				r = 1; //check this edge-edge, and normalB is ok to use for s_min calc.
			}
			else
			{
				//try negating edgeB->normal and seeing if that passes
				normalB[0] *= -1.0f;
				normalB[1] *= -1.0f;
				normalB[2] *= -1.0f;
				dot = vDotProduct(normalB, hullA->faces[(edge[0]->i_face[0])].normal);
				if(dot < 0.0f)
				{
					sign_eB_nA0 = 0;
				}
				else
				{
					sign_eB_nA0 = 1;
				}
				dot = vDotProduct(normalB, hullA->faces[(edge[0]->i_face[1])].normal);
				if(dot < 0.0f)
				{
					sign_eB_nA1 = 0;
				}
				else
				{
					sign_eB_nA1 = 1;
				}
				if(sign_eB_nA0 != sign_eB_nA1)
				{
					r = 1;
				}
				else
				{
					r = 0;
				}
			}
		}
	}
	else
	{
		
		if((sign_eA_nB0 != sign_eA_nB1) && (sign_eB_nA0 != sign_eB_nA1))
		{
			r = 1;	//check this edge-edge support feature
		}
		else
		{
			r = 0; //don't check this further
		}
	}

	return r;
}

//returns 1 if d_min was updated.
//note: I made this because I need to know what vertex was selected as the support feature.
static int CheckEdgePlane(struct box_struct * boxA, struct box_struct * boxB, float * axis, float * edgeOrigin, struct d_min_struct * d_min)
{
	struct box_collision_struct * hullA=0;
	struct box_collision_struct * hullB=0;
	float temp_vec[3];
	float normal_vec[3];
	float d;
	float dist;
	float min_dot;
	int i;
	int min_i;
	int r=0;

	hullA = &(boxA->hull);
	hullB = &(boxB->hull);

	normal_vec[0] = axis[0];
	normal_vec[1] = axis[1];
	normal_vec[2] = axis[2];
	
	normal_vec[0] *= -1.0f;
	normal_vec[1] *= -1.0f;
	normal_vec[2] *= -1.0f;

	//d = SATFindSupport(hullB, axis, edgeOrigin);
	for(i = 0; i < hullB->num_pos; i++)
	{
		d = vDotProduct(normal_vec, (hullB->positions+(i*3)));
		if(i == 0)
		{
			min_dot = d;
			min_i = i;
		}
		else
		{
			if(d < min_dot)
			{
				min_i = i;
				min_dot = d;
			}
		}
	}
	//support vertex index will be stored in min_i

	vSubtract(temp_vec, (hullB->positions+(min_i*3)), edgeOrigin);
	dist = vDotProduct(axis, temp_vec);

	if(d_min->is_initialized == 0)
	{
		d_min->d_min = dist;
		d_min->s_min[0] = axis[0];
		d_min->s_min[1] = axis[1];
		d_min->s_min[2] = axis[2];
		r = 1;
		d_min->is_initialized = 1;
	}
	else
	{
		if(dist > d_min->d_min)
		{
			d_min->d_min = dist;
			d_min->s_min[0] = axis[0];
			d_min->s_min[1] = axis[1];
			d_min->s_min[2] = axis[2];
			r = 1;
		}
	}

	return r;
}

static int CreateEdgeContact(struct d_min_struct * d_min, struct box_collision_struct * boxA, struct box_collision_struct * boxB, struct contact_manifold_struct * contact_manifold)
{
	struct edge_struct * edgeA=0;
	struct edge_struct * edgeB=0;
	float edgePosA[6];	//2 verts that make the edge
	float edgePosB[6];
	float d1343, d4321, d1321, d4343, d2121;
	float n, d;
	float edgeBPoint[3];	//closest point on edge B of line btwn edge A to edge B
	float edgeAPoint[3];	//closest point on edge A of line btwn edge A to edge B
	float mu_a;
	float mu_b;
	float unit[3];
	float mag;

	edgeA = boxA->edges + d_min->i_edge[0];
	edgeB = boxB->edges + d_min->i_edge[1];

	//fill in the edges for ease of use
	//note: d_min.i_hull is not set for edge pair
	edgePosA[0] = boxA->positions[(edgeA->i_vertices[0]*3)];
	edgePosA[1] = boxA->positions[(edgeA->i_vertices[0]*3)+1];
	edgePosA[2] = boxA->positions[(edgeA->i_vertices[0]*3)+2];

	edgePosA[3] = boxA->positions[(edgeA->i_vertices[1]*3)];
	edgePosA[4] = boxA->positions[(edgeA->i_vertices[1]*3)+1];
	edgePosA[5] = boxA->positions[(edgeA->i_vertices[1]*3)+2];

	edgePosB[0] = boxB->positions[(edgeB->i_vertices[0]*3)];
	edgePosB[1] = boxB->positions[(edgeB->i_vertices[0]*3)+1];
	edgePosB[2] = boxB->positions[(edgeB->i_vertices[0]*3)+2];

	edgePosB[3] = boxB->positions[(edgeB->i_vertices[1]*3)];
	edgePosB[4] = boxB->positions[(edgeB->i_vertices[1]*3)+1];
	edgePosB[5] = boxB->positions[(edgeB->i_vertices[1]*3)+2];

	d1343 = ((edgePosA[0] - edgePosB[0])*(edgePosB[3] - edgePosB[0])) 
		+ ((edgePosA[1] - edgePosB[1])*(edgePosB[4] - edgePosB[1])) 
		+ ((edgePosA[2] - edgePosB[2])*(edgePosB[5] - edgePosB[2]));

	d4321 = ((edgePosB[3] - edgePosB[0])*(edgePosA[3] - edgePosA[0])) 
		+ ((edgePosB[4] - edgePosB[1])*(edgePosA[4] - edgePosA[1])) 
		+ ((edgePosB[5] - edgePosB[2])*(edgePosA[5] - edgePosA[2]));

	d1321 = ((edgePosA[0] - edgePosB[0])*(edgePosA[3] - edgePosA[0])) 
		+ ((edgePosA[1] - edgePosB[1])*(edgePosA[4] - edgePosA[1])) 
		+ ((edgePosA[2] - edgePosB[2])*(edgePosA[5] - edgePosA[2]));

	d4343 = ((edgePosB[3] - edgePosB[0])*(edgePosB[3] - edgePosB[0])) 
		+ ((edgePosB[4] - edgePosB[1])*(edgePosB[4] - edgePosB[1])) 
		+ ((edgePosB[5] - edgePosB[2])*(edgePosB[5] - edgePosB[2]));

	d2121 = ((edgePosA[3] - edgePosA[0])*(edgePosA[3] - edgePosA[0])) 
		+ ((edgePosA[4] - edgePosA[1])*(edgePosA[4] - edgePosA[1])) 
		+ ((edgePosA[5] - edgePosA[2])*(edgePosA[5] - edgePosA[2]));

	//denominator of point on edgeB is 0, can't proceed
	if(fabs(d4343) < 0.0000001f)
		return 0;

	//denominator of point on edgeA is 0, can't proceed
	d = (d2121*d4343) - (d4321*d4321);
	if(fabs(d) < 0.0000001f)
		return 0;
	n = (d1343*d4321)-(d1321*d4343);
	mu_a = n/d;

	n = d1343 + (mu_a*d4321);
	mu_b = n/d4343;

	edgeAPoint[0] = edgePosA[0] + (mu_a*(edgePosA[3]-edgePosA[0]));
	edgeAPoint[1] = edgePosA[1] + (mu_a*(edgePosA[4]-edgePosA[1]));
	edgeAPoint[2] = edgePosA[2] + (mu_a*(edgePosA[5]-edgePosA[2]));

	edgeBPoint[0] = edgePosB[0] + (mu_b*(edgePosB[3] - edgePosB[0]));
	edgeBPoint[1] = edgePosB[1] + (mu_b*(edgePosB[4] - edgePosB[1]));
	edgeBPoint[2] = edgePosB[2] + (mu_b*(edgePosB[5] - edgePosB[2]));

	vSubtract(unit, edgePosB, edgePosA);
	mag = vMagnitude(unit);
	/*if(mag < 0.0000001f) //TODO: this check doesn't really do anything
	{
		contact_info->point[0] = edgeAPoint[0];
		contact_info->point[1] = edgeAPoint[1];
		contact_info->point[2] = edgeAPoint[2];

		contact_info->normal[0] = d_min->s_min[0];
		contact_info->normal[1] = d_min->s_min[1];
		contact_info->normal[2] = d_min->s_min[2];
	}*/
	vNormalize(unit);

	unit[0] *= 0.5f;
	unit[1] *= 0.5f;
	unit[2] *= 0.5f;

	contact_manifold->num_contacts = 1;
	contact_manifold->contacts[0].point[0] = edgeAPoint[0] + (unit[0]*(edgeBPoint[0]-edgeAPoint[0]));
	contact_manifold->contacts[0].point[1] = edgeAPoint[1] + (unit[1]*(edgeBPoint[1]-edgeAPoint[1]));
	contact_manifold->contacts[0].point[2] = edgeAPoint[2] + (unit[2]*(edgeBPoint[2]-edgeAPoint[2]));

	contact_manifold->contacts[0].normal[0] = d_min->s_min[0];
	contact_manifold->contacts[0].normal[1] = d_min->s_min[1];
	contact_manifold->contacts[0].normal[2] = d_min->s_min[2];

	contact_manifold->contacts[0].penetration = d_min->d_min;

	return 1;
}

static int CreateFaceContact(struct d_min_struct * d_min, 
	struct box_collision_struct * boxA, 
	struct box_collision_struct * boxB, 
	struct contact_manifold_struct * contact_manifold)
{
	//d_min struct holds which face is reference face and which is incident face based
	//on SATCheckDirection().
	//i_hull[0] -> reference face
	//i_hull[1] -> incident face
	float smallest_dot;
	float dot;
	float prev_vertexPosList[12];
	float next_vertexPosList[12];
	float * vertexPosLists[2];		//holds the two vertex lists of clipped vertices.
	int i_nextList;	//index in vertexPosLists that points to the next list
	int i_prevList;	//index in vertexPosLists that points to the prev list
	int list_numVerts[2];			//number of verts in each vertex list.
	float * p_prevVert;
	float * p_nextVert;
	float clipPlaneNormal[3];
	float clipPlaneEdge[3];
	float * clipPlaneVerts[2];
	float clipDirVec[3];
	float clip_dist;
	float temp_vec[3];
	float contact_pos_array[12];
	struct box_collision_struct * referenceHull=0;
	struct box_collision_struct * incidentHull=0;
	struct face_struct * referenceFace=0;
	struct face_struct * incidentFace=0;
	int i;
	int j;
	int i_nextVert;
	int i_incidentFace;

	//setup two lists of vertex positions for clipping
	vertexPosLists[0] = prev_vertexPosList;
	vertexPosLists[1] = next_vertexPosList;
	list_numVerts[0] = 0;
	list_numVerts[1] = 0;
	i_nextList = 1;
	i_prevList = 0;

	if(d_min->i_hull[0] == 0) //reference face is from A
	{
		referenceHull = boxA;
		incidentHull = boxB;
	}
	if(d_min->i_hull[0] == 1) //reference face is from B
	{
		referenceHull = boxB;
		incidentHull = boxA;
	}

	referenceFace = referenceHull->faces + d_min->i_face;

	//need to identify incident face on the other hull. Loop through all faces on the other hull
	//looking for a face with smallest dot product with reference face.
	for(i = 0; i < incidentHull->num_faces; i++)
	{
		if(i == 0)
		{
			smallest_dot = vDotProduct(referenceFace->normal, incidentHull->faces[i].normal);
			incidentFace = (incidentHull->faces+i);
		}
		else
		{
			dot = vDotProduct(referenceFace->normal, incidentHull->faces[i].normal);
			if(dot < smallest_dot)
			{
				smallest_dot = dot;
				incidentFace = (incidentHull->faces+i);
			}
		}
	}

	//Setup the vertex list. Add all vertices of the incident face to the prev vertex pos list.
	//The clipping will place new vertices in the i_nextList
	for(i = 0; i < 4; i++)
	{
		(vertexPosLists[0])[(i*3)] = incidentHull->positions[((incidentFace->i_vertices[i])*3)];
		(vertexPosLists[0])[(i*3)+1] = incidentHull->positions[((incidentFace->i_vertices[i])*3)+1];
		(vertexPosLists[0])[(i*3)+2] = incidentHull->positions[((incidentFace->i_vertices[i])*3)+2];
	}
	list_numVerts[0] = 4;

	//loop through each edge of the reference face
	for(i = 0; i < 4; i++)
	{
		list_numVerts[i_nextList] = 0; //reset the next vertex list.

		//create a clip plane from the current reference face edge.
		clipPlaneVerts[0] = &(referenceHull->positions[(referenceFace->i_vertices[i]*3)]);
		i_nextVert = (i+1) % 4; //wrap back to 0
		clipPlaneVerts[1] = &(referenceHull->positions[(referenceFace->i_vertices[i_nextVert]*3)]);

		vSubtract(clipPlaneEdge, clipPlaneVerts[1], clipPlaneVerts[0]);
		vCrossProduct(clipPlaneNormal, referenceFace->normal, clipPlaneEdge); //clipPlaneNormal should face inward towards center of ref plane
		vNormalize(clipPlaneNormal);

		for(j = 0; j < list_numVerts[i_prevList]; j++)
		{
			p_prevVert = vertexPosLists[i_prevList]+(j*3);
			vSubtract(temp_vec, p_prevVert, clipPlaneVerts[0]);
			dot = vDotProduct(clipPlaneNormal, temp_vec);
			p_nextVert = vertexPosLists[i_nextList]+(list_numVerts[i_nextList]*3);

			//if the dot product is >= 0 then leave the vertex as is, but if it is
			//negative, then the vertex is on the side of the clip plane and needs
			//to be clipped.
			if(dot >= 0.0f)
			{
				p_nextVert[0] = p_prevVert[0];
				p_nextVert[1] = p_prevVert[1];
				p_nextVert[2] = p_prevVert[2];
				list_numVerts[i_nextList] += 1;
			}
			else //clip the vertex
			{
				clipDirVec[0] = clipPlaneNormal[0];
				clipDirVec[1] = clipPlaneNormal[1];
				clipDirVec[2] = clipPlaneNormal[2];
				clip_dist = fabs(dot);
				clipDirVec[0] *= clip_dist;
				clipDirVec[1] *= clip_dist;
				clipDirVec[2] *= clip_dist;

				vAdd(p_nextVert, p_prevVert, clipDirVec);
				list_numVerts[i_nextList] += 1;
			}
		}
		
		//advance i_prevList, i_nextList
		i_prevList = (i_prevList + 1) % 2;
		i_nextList = (i_nextList + 1) % 2;
	}
	//now the final list of vertices is in vertexPosLists[i_prevList]

	//now take all contacts and keep the ones below the reference frame
	list_numVerts[i_nextList] = 0;
	for(i = 0; i < list_numVerts[i_prevList]; i++)
	{
		p_prevVert = vertexPosLists[i_prevList] + (i*3); //get the ith vertex from the list
		p_nextVert = contact_pos_array + (list_numVerts[i_nextList]*3);
		clipPlaneVerts[0] = referenceHull->positions + (referenceFace->i_vertices[0]*3); //get a vertex on the reference face.

		vSubtract(temp_vec, p_prevVert, clipPlaneVerts[0]);
		dot = vDotProduct(temp_vec, referenceFace->normal);

		if(dot <= 0.0f) //plane normal vec points out, so look for negative dot-products, these are below the plane
		{
			//for vertices below the clip plane move them to reference plane
			clipDirVec[0] = referenceFace->normal[0];
			clipDirVec[1] = referenceFace->normal[1];
			clipDirVec[2] = referenceFace->normal[2];
			clip_dist = fabs(dot);

			//save the clip_dist in the contact_info_struct as penetration
			//TODO: Is this the best way to get penetration?
			contact_manifold->contacts[(list_numVerts[i_nextList])].penetration = clip_dist;

			clipDirVec[0] *= clip_dist;
			clipDirVec[1] *= clip_dist;
			clipDirVec[2] *= clip_dist;
			vAdd(p_nextVert, p_prevVert, clipDirVec);
			
			list_numVerts[i_nextList] += 1;
		}
	}
	
	//transfer contact_pos_array to contact_info_struct. fill in new num_contacts too.
	contact_manifold->num_contacts = list_numVerts[i_nextList];
	for(i = 0; i < contact_manifold->num_contacts; i++)
	{
		contact_manifold->contacts[i].point[0] = contact_pos_array[(i*3)];
		contact_manifold->contacts[i].point[1] = contact_pos_array[(i*3)+1];
		contact_manifold->contacts[i].point[2] = contact_pos_array[(i*3)+2];
		contact_manifold->contacts[i].normal[0] = d_min->s_min[0];
		contact_manifold->contacts[i].normal[1] = d_min->s_min[1];
		contact_manifold->contacts[i].normal[2] = d_min->s_min[2];
	}

	//perform a check for contacts that are too close together
	
	return 1;
}

//UpdateHull transforms the hull to the current orientation/position of the box.
static void UpdateHull(struct box_collision_struct * base_hull, struct box_struct * box)
{
	struct box_collision_struct * hull;
	int i;

	hull = &(box->hull);

	//Transform the copy of the hull to world coordinates
	for(i = 0; i < hull->num_pos; i++)
	{
		hull->positions[(i*3)] = base_hull->positions[(i*3)];
		hull->positions[(i*3)+1] = base_hull->positions[(i*3)+1];
		hull->positions[(i*3)+2] = base_hull->positions[(i*3)+2];
		mmTransformVec3(box->orientation, (hull->positions+(i*3)));
		hull->positions[(i*3)] += box->pos[0];
		hull->positions[(i*3)+1] += box->pos[1];
		hull->positions[(i*3)+2] += box->pos[2];
	}
	for(i = 0; i < hull->num_faces; i++)
	{
		hull->faces[i].normal[0] = base_hull->faces[i].normal[0];
		hull->faces[i].normal[1] = base_hull->faces[i].normal[1];
		hull->faces[i].normal[2] = base_hull->faces[i].normal[2];
		mmTransformVec3(box->orientation, hull->faces[i].normal);
	}
	for(i = 0; i < hull->num_edges; i++)
	{
		hull->edges[i].normal[0] = base_hull->edges[i].normal[0];
		hull->edges[i].normal[1] = base_hull->edges[i].normal[1];
		hull->edges[i].normal[2] = base_hull->edges[i].normal[2];
		mmTransformVec3(box->orientation, hull->edges[i].normal);
	}
}

static int CopyHull(struct box_collision_struct * dest, struct box_collision_struct * src)
{
	int i;

	//positions
	dest->positions = (float*)malloc(src->num_pos*3*sizeof(float));
	if(dest->positions == 0)
		return 0;
	dest->num_pos = src->num_pos;
	for(i = 0; i < src->num_pos; i++)
	{
		dest->positions[(i*3)] = src->positions[(i*3)];
		dest->positions[(i*3)+1] = src->positions[(i*3)+1];
		dest->positions[(i*3)+2] = src->positions[(i*3)+2];
	}

	//faces
	dest->faces = (struct face_struct*)malloc(src->num_faces*sizeof(struct face_struct));
	if(dest->faces == 0)
		return 0;
	dest->num_faces = src->num_faces;
	for(i = 0; i < src->num_faces; i++)
	{
		memcpy(dest->faces+i, src->faces+i, sizeof(struct face_struct));
	}

	//edges
	dest->edges = (struct edge_struct*)malloc(src->num_edges*sizeof(struct edge_struct));
	if(dest->edges == 0)
		return 0;
	dest->num_edges = src->num_edges;
	for(i = 0; i < src->num_edges; i++)
	{
		memcpy(dest->edges+i, src->edges+i, sizeof(struct edge_struct));
	}

	return 1;
}

static void HandleKeyboardInput(Display * dpy)
{
	struct d_min_struct d_min;
	struct contact_info_struct contact_array[4];
	char keys_return[32];
	float fcamera_speed = 0.001f;
	static int g_key_is_up;
	static int spacebar_key_is_down;
	static int plus_key_is_down;
	int num_contacts=4;
	int r;

	XQueryKeymap(dpy, keys_return);

	//up-arrow
	/*if(CheckKey(keys_return, 111) == 1)
	{
		//make up-arrow move box B
		g_a_box[0].pos[1] += 0.001f;
	}

	//down-arrow
	if(CheckKey(keys_return, 116) == 1)
	{
		//make down-arrow move box B
		g_a_box[0].pos[1] -= 0.001f;
	}*/


	//'g' key.
	/*if(CheckKey(keys_return, 42) == 1)
	{
		if(g_key_is_up == 0)
		{
			r = FindSeparatingAxis(&(g_a_box[0]), &(g_a_box[1]), &d_min);
			//printf("separating axis=%d\n", r);

			CreateFaceContact(&d_min, 
					&(g_a_box[0].hull), 
					&(g_a_box[1].hull), 
					contact_array, 
					&num_contacts);
		}
		g_key_is_up = 1;
	}
	else
	{
		g_key_is_up = 0;
	}*/

	//'spacebar' key
	if(CheckKey(keys_return, 65) == 1)
	{
		if(spacebar_key_is_down == 0)
		{
			SimulationStep();
		}
		spacebar_key_is_down = 1;
	}
	else
	{
		spacebar_key_is_down = 0;
	}

	//'+' key
	if(CheckKey(keys_return, 21) == 1)
	{
		if(plus_key_is_down == 0)
		{
			g_simulation_run = (g_simulation_run + 1) % 2; //cycle g_simulation_run
			printf("g_simulation_run=%d", g_simulation_run);
		}
		plus_key_is_down = 1;
	}
	else
	{
		plus_key_is_down = 0;
	}

	//'w' key
	if(CheckKey(keys_return, 25) == 1)
	{
		g_neg_camera_pos[2] -= fcamera_speed;
	}
	//'s' key
	if(CheckKey(keys_return, 39) == 1)
	{
		g_neg_camera_pos[2] += fcamera_speed;
	}
	//'a' key
	if(CheckKey(keys_return, 38) == 1)
	{
		g_neg_camera_pos[0] += fcamera_speed;
	}
	//'d' key
	if(CheckKey(keys_return, 40) == 1)
	{
		g_neg_camera_pos[0] -= fcamera_speed;
	}
	//shift key
	if(CheckKey(keys_return, 50) == 1)
	{
		g_neg_camera_pos[1] -= fcamera_speed;
	}
	//ctrl key
	if(CheckKey(keys_return, 37) == 1)
	{
		g_neg_camera_pos[1] += fcamera_speed;
	}

	//up arrow key 
	if(CheckKey(keys_return, 111) == 1)
	{
		g_neg_camera_rot[0] += 0.01f;
	}
	//down arrow key
	if(CheckKey(keys_return, 116) == 1)
	{
		g_neg_camera_rot[0] -= 0.01f;
	}
	//left arrow key 
	if(CheckKey(keys_return, 113) == 1)
	{
		g_neg_camera_rot[1] -= 0.01f;
	}
	//right arrow key
	if(CheckKey(keys_return, 114) == 1)
	{
		g_neg_camera_rot[1] += 0.01f;
	}
	//'z' key
	if(CheckKey(keys_return, 52) == 1)
	{
		g_neg_camera_rot[0] = 0.0f;
		g_neg_camera_rot[1] = 0.0f;
	}
}

/*
hint: use the program 'xev' to print xevents to find new keycodes
*/
int CheckKey(char * keys_return, int key_bit_index)
{
	int i;
	int bit;
	int mask=1;
	int r=0;
	
	i = key_bit_index/8;
	bit = key_bit_index % 8;
	mask = mask << bit;
	
	if((keys_return[i] & mask) != 0)
	{
		r = 1;
	}
	return r;
}
