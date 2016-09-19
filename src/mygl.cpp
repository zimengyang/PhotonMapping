#include "mygl.h"
#include <la.h>

#include <iostream>
#include <QApplication>
#include <QKeyEvent>
#include <QXmlStreamReader>
#include <QFileDialog>
#include <renderthread.h>
#include <raytracing/samplers/stratifiedpixelsampler.h>
#include <raytracing/photonintegrator.h>
#include <QtOpenGL/QGLWidget>
#include <QLabel>
#include <QOpenGLTexture>

GLfloat vertexData[] = {
    //  X     Y     Z       U     V
     -1.f, -1.0f, 0.0f,   0.0f, 1.0f,
    -1.0f,1.0f, 0.0f,   0.0f, 0.0f,
     1.0f,-1.0f, 0.0f,   1.0f, 1.0f,
     1.0f,1.0f, 0.0f,   1.0f, 0.0f,
};

MyGL::MyGL(QWidget *parent)
    : GLWidget277(parent)
{
    setFocusPolicy(Qt::ClickFocus);
    render_threads = NULL;
}

MyGL::~MyGL()
{
    makeCurrent();

    vao.destroy();

    if(integrator != NULL) delete integrator;
}

void MyGL::initializeGL()
{
    // Create an OpenGL context
    initializeOpenGLFunctions();
    // Print out some information about the current OpenGL context
    debugContextVersion();

    // Set a few settings/modes in OpenGL rendering
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    // Set the size with which points should be rendered
    glPointSize(5);
    // Set the color with which the screen is filled at the start of each render call.
    glClearColor(0.5, 0.5, 0.5, 1);

    printGLErrorLog();

    // Create a Vertex Attribute Object
    vao.create();

    // Create and set up the diffuse shader
    prog_lambert.create(":/glsl/lambert.vert.glsl", ":/glsl/lambert.frag.glsl");
    // Create and set up the flat-color shader
    prog_flat.create(":/glsl/flat.vert.glsl", ":/glsl/flat.frag.glsl");

    // set progressive render glsl shader
    progressive_render.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/glsl/progressive.vert.glsl");
    progressive_render.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/glsl/progressive.frag.glsl");
    progressive_render.link();

    // We have to have a VAO bound in OpenGL 3.2 Core. But if we're not
    // using multiple VAOs, we can just bind one once.
    vao.bind();

    //Test scene data initialization
    scene.CreateTestScene();
    integrator = new Integrator();
    integrator->scene = &scene;
    integrator->intersection_engine = &intersection_engine;
    intersection_engine.scene = &scene;
    ResizeToSceneCamera();

    // test scene build BVH tree
    intersection_engine.root = BVHNode::buildBVHTree(scene.objects);

    currentState = Preview;

    //set point array buffer
    glGenBuffers(1,&vertexBufferId);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertexData),vertexData,GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void MyGL::resizeGL(int w, int h)
{
    gl_camera = Camera(w, h);

    glm::mat4 viewproj = gl_camera.getViewProj();

    // Upload the projection matrix
    prog_lambert.setViewProjMatrix(viewproj);
    prog_flat.setViewProjMatrix(viewproj);

    printGLErrorLog();
}

//This function is called by Qt any time your GL window is supposed to update
//For example, when the function updateGL is called, paintGL is called implicitly.
void MyGL::paintGL()
{
    if(currentState == Preview)
    {
        // Clear the screen so that we only see newly drawn images
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Update the viewproj matrix
        prog_lambert.setViewProjMatrix(gl_camera.getViewProj());
        prog_flat.setViewProjMatrix(gl_camera.getViewProj());
        GLDrawScene();
    }
    else if(currentState == Rendering)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        progressive_render.bind();

        QOpenGLTexture texture(gfb);
        texture.setMagnificationFilter(QOpenGLTexture::Linear);
        texture.setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
        texture.bind();

        glBindBuffer(GL_ARRAY_BUFFER,vertexBufferId);

        // connect the xyz to the "vert" attribute of the vertex shader
        glEnableVertexAttribArray(progressive_render.attributeLocation("vert"));
        glVertexAttribPointer(progressive_render.attributeLocation("vert"), 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), NULL);

        // connect the uv coords to the "vertTexCoord" attribute of the vertex shader
        glEnableVertexAttribArray(progressive_render.attributeLocation("vertTexCoord"));
        glVertexAttribPointer(progressive_render.attributeLocation("vertTexCoord"), 2, GL_FLOAT, GL_TRUE,  5*sizeof(GLfloat), (const GLvoid*)(3 * sizeof(GLfloat)));

        glDrawArrays(GL_TRIANGLE_STRIP,0,4);
        texture.destroy();
    }

}

void MyGL::drawBVHTree(BVHNode *root, int level)
{
    if(root == NULL)
        return;

    if(root->left != NULL)
        drawBVHTree(root->left,level);

    // render first several level
    if(root->depth <= level)
    {
        prog_flat.draw(*this, root->bBox);
    }

    if(root->right != NULL)
        drawBVHTree(root->right,level);
}

void MyGL::GLDrawScene()
{
    for(Geometry *g : scene.objects)
    {
        if(g->drawMode() == GL_TRIANGLES)
        {
            prog_lambert.setModelMatrix(g->transform.T());
            prog_lambert.draw(*this, *g);
        }
        else if(g->drawMode() == GL_LINES)
        {
            prog_flat.setModelMatrix(g->transform.T());
            prog_flat.draw(*this, *g);
        }
    }
    for(Geometry *l : scene.lights)
    {
        prog_flat.setModelMatrix(l->transform.T());
        prog_flat.draw(*this, *l);
    }
    prog_flat.setModelMatrix(glm::mat4(1.0f));
    prog_flat.draw(*this, scene.camera);

    //Recursively traverse the BVH hierarchy stored in the intersection engine and draw each node
    drawBVHTree(intersection_engine.root, 10);
}

void MyGL::ResizeToSceneCamera()
{
    this->setFixedWidth(scene.camera.width);
    this->setFixedHeight(scene.camera.height);
    gl_camera = Camera(scene.camera);
}

void MyGL::keyPressEvent(QKeyEvent *e)
{
    float amount = 2.0f;
    if(e->modifiers() & Qt::ShiftModifier){
        amount = 10.0f;
    }
    // http://doc.qt.io/qt-5/qt.html#Key-enum
    if (e->key() == Qt::Key_Escape) {
        QApplication::quit();
    } else if (e->key() == Qt::Key_Right) {
        gl_camera.RotateAboutUp(-amount);
    } else if (e->key() == Qt::Key_Left) {
        gl_camera.RotateAboutUp(amount);
    } else if (e->key() == Qt::Key_Up) {
        gl_camera.RotateAboutRight(-amount);
    } else if (e->key() == Qt::Key_Down) {
        gl_camera.RotateAboutRight(amount);
    } else if (e->key() == Qt::Key_1) {
        gl_camera.fovy += amount;
    } else if (e->key() == Qt::Key_2) {
        gl_camera.fovy -= amount;
    } else if (e->key() == Qt::Key_W) {
        gl_camera.TranslateAlongLook(amount);
    } else if (e->key() == Qt::Key_S) {
        gl_camera.TranslateAlongLook(-amount);
    } else if (e->key() == Qt::Key_D) {
        gl_camera.TranslateAlongRight(amount);
    } else if (e->key() == Qt::Key_A) {
        gl_camera.TranslateAlongRight(-amount);
    } else if (e->key() == Qt::Key_Q) {
        gl_camera.TranslateAlongUp(-amount);
    } else if (e->key() == Qt::Key_E) {
        gl_camera.TranslateAlongUp(amount);
    } else if (e->key() == Qt::Key_F) {
        gl_camera.CopyAttributes(scene.camera);
    } else if (e->key() == Qt::Key_R) {
        scene.camera = Camera(gl_camera);
        scene.camera.recreate();
    } else if (e->key() == Qt::Key_P) {
        scene.film.WriteImage(filePath);
    }
    gl_camera.RecomputeAttributes();
    update();  // Calls paintGL, among other things
}

void MyGL::SceneLoadDialog()
{
    currentState = Preview;

    QString filepath = QFileDialog::getOpenFileName(0, QString("Load Scene"), QString("../scene_files"), tr("*.xml"));
    if(filepath.length() == 0)
    {
        return;
    }

    QFile file(filepath);
    int i = filepath.length() - 1;
    while(QString::compare(filepath.at(i), QChar('/')) != 0)
    {
        i--;
    }
    QStringRef local_path = filepath.leftRef(i+1);
    //Reset all of our objects
    scene.Clear();
    if(integrator != NULL) delete integrator;
    integrator = NULL;

    //release bvhtree space
    BVHNode::releaseTree(intersection_engine.root);

    intersection_engine = IntersectionEngine();
    //Load new objects based on the XML file chosen.
    xml_reader.LoadSceneFromFile(file, local_path, scene, integrator);
    integrator->scene = &scene;
    integrator->intersection_engine = &intersection_engine;
    intersection_engine.scene = &scene;

    //create BvhTree
    intersection_engine.root = BVHNode::buildBVHTree(scene.objects);

    // is integrator a photon mapping integrator? do more preparation
    PhotonIntegrator* photonIntegrator = dynamic_cast<PhotonIntegrator*>(integrator);
    if(photonIntegrator)
    {
        // other stuffs
        std::cout<<"It's a photon integrator\n";
        photonIntegrator->Preprocessing();
    }

    ResizeToSceneCamera();
    update();
}

void MyGL::RaytraceScene()
{
    // set opengl shader state
    gfb = grabFramebuffer();
    gfb = gfb.scaled(QSize(scene.camera.width,scene.camera.height));

    QString filepath = QFileDialog::getSaveFileName(0, QString("Save Image"), QString("../rendered_images"), tr("*.bmp"));
    if(filepath.length() == 0)
    {
        return;
    }
    filePath = filepath;
    currentState = Rendering;

#define MULTITHREADED 1
#if MULTITHREADED
    int sqrtThreadSize = 4;
    //Set up 16 (max) threads
    unsigned int width = scene.camera.width;
    unsigned int height = scene.camera.height;
    unsigned int x_block_size = (width >= sqrtThreadSize ? width/sqrtThreadSize : 1);
    unsigned int y_block_size = (height >= sqrtThreadSize ? height/sqrtThreadSize : 1);
    unsigned int x_block_count = width > sqrtThreadSize ? width/x_block_size : 1;
    unsigned int y_block_count = height > sqrtThreadSize ? height/y_block_size : 1;
    if(x_block_count * x_block_size < width) x_block_count++;
    if(y_block_count * y_block_size < height) y_block_count++;

    //unsigned int num_render_threads = x_block_count * y_block_count;
    //RenderThread **render_threads = new RenderThread*[num_render_threads];

    num_render_threads = x_block_count * y_block_count;
    render_threads = new RenderThread*[num_render_threads];

    //Launch the render threads we've made
    for(unsigned int Y = 0; Y < y_block_count; Y++)
    {
        //Compute the columns of the image that the thread should render
        unsigned int y_start = Y * y_block_size;
        unsigned int y_end = glm::min((Y + 1) * y_block_size, height);
        for(unsigned int X = 0; X < x_block_count; X++)
        {
            //Compute the rows of the image that the thread should render
            unsigned int x_start = X * x_block_size;
            unsigned int x_end = glm::min((X + 1) * x_block_size, width);
            //Create and run the thread
            render_threads[Y * x_block_count + X] = new RenderThread(x_start, x_end, y_start, y_end, scene.sqrt_samples, integrator->getMaxDepth(), &(scene.film), &(scene.camera), integrator, &gfb);
            render_threads[Y * x_block_count + X]->start();
        }
    }

//    bool still_running;
//    do
//    {
//        still_running = false;
//        for(unsigned int i = 0; i < num_render_threads; i++)
//        {
//            if(render_threads[i]->isRunning())
//            {
//                //manageRender();
//                still_running = true;
//                break;
//            }
//        }
//        if(still_running)
//        {
//            //Free the CPU to let the remaining render threads use it
//            QThread::yieldCurrentThread();
//        }
//    }
//    while(still_running);

//    //Finally, clean up the render thread objects
//    for(unsigned int i = 0; i < num_render_threads; i++)
//    {
//        delete render_threads[i];
//    }
//    delete [] render_threads;

#else
    StratifiedPixelSampler pixel_sampler(scene.sqrt_samples,rand());
    for(unsigned int i = 0; i < scene.camera.width; i++)
    {
        for(unsigned int j = 0; j < scene.camera.height; j++)
        {
            QList<glm::vec2> sample_points = pixel_sampler.GetSamples(i, j);
            glm::vec3 accum_color;
            for(int a = 0; a < sample_points.size(); a++)
            {
                glm::vec3 color = integrator->TraceRay(scene.camera.Raycast(sample_points[a]), 0);
                accum_color += color;
            }
            scene.film.pixels[i][j] = accum_color / (float)sample_points.size();
        }
    }
#endif
    //scene.film.WriteImage(filepath);

    /*
     * dispatch a single slot first time, update the render window automaticly
     */
    QTimer::singleShot(500,this,SLOT(updateRender()));

    //currentState = Preview;
}

void MyGL::updateRender()
{
    bool still_running ;
    still_running = false;

    myUpdate();

    for(unsigned int i = 0; i < num_render_threads; i++)
    {
        if(render_threads[i]->isRunning())
        {
            still_running = true;
            QTimer::singleShot(500,this,SLOT(updateRender()));
            return;
        }
    }

    //Finally, clean up the render thread objects
    if(!still_running)
    {
        //write image after all threads finish rendering
        //scene.film.WriteImage(filePath);
        for(unsigned int i = 0; i < num_render_threads; i++)
        {
            delete render_threads[i];
        }
        delete [] render_threads;
    }
}
