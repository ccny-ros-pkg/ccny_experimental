#include "glwidget.h"
#include <GL/glut.h>


GLWidget::GLWidget(QWidget *parent) :
    QGLWidget(parent)
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(updateGL()) );
    timer.start(16);

}



void GLWidget:: initializeGL(){
    glClearColor(.2, .2, .2, 1);
 //   glEnable(GL_DEPTH_TEST);
   // glEnable(GL_LIGHT0);
    //glEnable(GL_LIGHTING);

}


void  GLWidget:: paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glRotated(.5, 1, 1, 1);
    //glutWireTeapot(1);

    //glutWireDodecahedron();

    glutWireTorus(.40, .90, 5, 10);
    glViewport(0,0,321,251);

    /**
    glColor3f(1, 0,0);

    glBegin(GL_TRIANGLES);
        glVertex3d(3.14, 5, 6.77);
        glVertex3d(23.14, 25, 26.77);
        glVertex3d(13.14, 15, 10);
    glEnd();*/
}

void  GLWidget::resizeGL(int w, int h){}
