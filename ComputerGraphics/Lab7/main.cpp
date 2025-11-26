#include <GL/glut.h>
#include <cmath>
#include <iostream>

const double PI = 3.14159265358979323846;

float angleX = 0.0f;
float angleY = 0.0f;
int lastMouseX, lastMouseY;
bool isWireframe = true;

void drawCylinder(float rBase, float rTop, float height, int slices) {
    float halfH = height / 2.0f;
    
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= slices; ++i) {
        float angle = 2.0f * PI * i / slices;
        float x = cos(angle);
        float z = sin(angle);

        glNormal3f(x, 0.0f, z); 
        
        glVertex3f(rTop * x, halfH, rTop * z);
        glVertex3f(rBase * x, -halfH, rBase * z);
    }
    glEnd();

    if (!isWireframe) {
        glBegin(GL_POLYGON);
        glNormal3f(0.0f, 1.0f, 0.0f);
        for (int i = 0; i < slices; ++i) {
            float angle = 2.0f * PI * i / slices;
            glVertex3f(rTop * cos(angle), halfH, rTop * sin(angle));
        }
        glEnd();

        glBegin(GL_POLYGON);
        glNormal3f(0.0f, -1.0f, 0.0f);
        for (int i = 0; i < slices; ++i) {
            float angle = 2.0f * PI * i / slices;
            glVertex3f(rBase * cos(angle), -halfH, rBase * sin(angle));
        }
        glEnd();
    }
}

void drawBase() {
    glPushMatrix();
    glColor3f(0.4f, 0.4f, 0.4f);
    glTranslatef(0.0f, -2.5f, 0.0f);
    drawCylinder(0.8f, 0.4f, 1.5f, 20);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.0f, -3.25f, 0.0f);
    drawCylinder(1.0f, 1.0f, 0.1f, 20);
    glPopMatrix();
}

void drawMainTank() {
    glPushMatrix();
    if (isWireframe) glColor3f(0.0f, 0.8f, 1.0f);
    else {
        glColor4f(0.5f, 0.8f, 1.0f, 0.4f);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    drawCylinder(1.0f, 1.0f, 3.5f, 24);
    
    if (!isWireframe) glDisable(GL_BLEND);
    glPopMatrix();

    glPushMatrix();
    glColor3f(0.8f, 0.6f, 0.0f);
    glScalef(0.6f, 0.6f, 0.6f);
    drawCylinder(1.0f, 1.0f, 2.0f, 16);
    glPopMatrix();
}

void drawTopFunnel() {
    glPushMatrix();
    glColor3f(0.7f, 0.7f, 0.7f);
    glTranslatef(0.0f, 2.5f, 0.0f);
    drawCylinder(0.3f, 0.8f, 1.5f, 20);
    glTranslatef(0.0f, 0.8f, 0.0f);
    drawCylinder(1.0f, 1.0f, 0.1f, 20);
    glPopMatrix();
}

void drawFan() {
    glPushMatrix();
    glTranslatef(0.8f, 0.0f, 0.8f);
    glRotatef(45.0f, 0.0f, 1.0f, 0.0f);
    glRotatef(-30.0f, 1.0f, 0.0f, 0.0f);

    glColor3f(0.2f, 0.2f, 0.2f);
    glPushMatrix();
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    drawCylinder(0.1f, 0.1f, 1.0f, 10);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.0f, 0.0f, 0.5f);
    glColor3f(0.8f, 0.2f, 0.2f);
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    drawCylinder(0.3f, 0.0f, 0.6f, 16);
    glPopMatrix();

    for (int i = 0; i < 2; i++) {
        glPushMatrix();
        glTranslatef(0.0f, 0.0f, 0.4f);
        glRotatef(180.0f * i, 0.0f, 0.0f, 1.0f);
        
        glTranslatef(1.2f, 0.0f, 0.0f);
        glScalef(2.0f, 0.4f, 0.05f);
        glColor3f(0.9f, 0.9f, 0.9f);
        drawCylinder(0.5f, 0.5f, 1.0f, 12);
        
        glPopMatrix();
    }

    glPopMatrix();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glTranslatef(0.0f, 0.0f, -12.0f);
    glRotatef(angleX, 1.0f, 0.0f, 0.0f);
    glRotatef(angleY, 0.0f, 1.0f, 0.0f);

    if (isWireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_LIGHTING);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_LIGHTING);
    }

    drawBase();
    drawMainTank();
    drawTopFunnel();
    drawFan();

    glutSwapBuffers();
}

void init() {
    glEnable(GL_DEPTH_TEST);
    
    glEnable(GL_LIGHT0);
    GLfloat lightPos[] = { 10.0f, 10.0f, 10.0f, 1.0f };
    GLfloat lightDiff[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiff);
    glEnable(GL_COLOR_MATERIAL);

    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)w / (double)h, 1.0, 200.0);
    glMatrixMode(GL_MODELVIEW);
}

void mouseMotion(int x, int y) {
    angleY += (x - lastMouseX) * 0.5f;
    angleX += (y - lastMouseY) * 0.5f;
    lastMouseX = x;
    lastMouseY = y;
    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
    lastMouseX = x;
    lastMouseY = y;
}

void keyboard(unsigned char key, int x, int y) {
    if (key == 'w' || key == 'W') isWireframe = !isWireframe;

    if (key == 27) exit(0);
    glutPostRedisplay();
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Lab 7: 3D Object Structure (Fan Variant 22)");

    init();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMotionFunc(mouseMotion);
    glutMouseFunc(mouse);
    glutKeyboardFunc(keyboard);

    std::cout << "Controls:\n";
    std::cout << " [Mouse Drag] - Rotate Object\n";
    std::cout << " [W] - Toggle Wireframe/Solid Mode\n";
    std::cout << " [ESC] - Exit\n";

    glutMainLoop();
    return 0;
}

