#pragma once

#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>

#include <openGL/glwidget277.h>
#include <la.h>
#include <openGL/shaderprogram.h>
#include <scene/camera.h>
#include <scene/scene.h>
#include <scene/xmlreader.h>
#include <raytracing/integrator.h>
#include <scene/xmlreader.h>
#include <raytracing/integrator.h>
#include <qpainter.h>
#include <renderthread.h>

class RenderThread;

class MyGL
    : public GLWidget277
{
    Q_OBJECT
private:
    QOpenGLVertexArrayObject vao;

    ShaderProgram prog_lambert;
    ShaderProgram prog_flat;
    QOpenGLShaderProgram progressive_render;

    Camera gl_camera;//This is a camera we can move around the scene to view it from any angle.
                                //However, the camera defined in the config file is the one from which the scene will be rendered.
                                //If you move gl_camera, you will be able to see the viewing frustum of the scene's camera.

    Scene scene;
    XMLReader xml_reader;
    Integrator* integrator;
    IntersectionEngine intersection_engine;

public:
    explicit MyGL(QWidget *parent = 0);
    ~MyGL();

    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    //void paintEvent(QPaintEvent* e);

    void SceneLoadDialog();
    void GLDrawScene();
    void ResizeToSceneCamera();

    void RaytraceScene();

    //unsigned char* pixBuffer;
    QImage gfb; // frame buffer
    QImage gldata; // converted to  gl version
    enum State {Rendering,Preview};
    State currentState;
    GLuint textureID;

    GLuint vertexBufferId;

    void drawBVHTree(BVHNode* root,int level);

    QString filePath;
    void myUpdate() {update();}

    RenderThread** render_threads;
    unsigned int num_render_threads;

protected:
    void keyPressEvent(QKeyEvent *e);

signals:
    void sig_ResizeToCamera(int,int);

public slots:
    void updateRender();
    //void updateRender(RenderThread** render_threads,unsigned int num_render_threads);

};
