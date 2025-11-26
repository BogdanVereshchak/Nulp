#include <windows.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <ctime>

using namespace std;

const double PI = 3.14159265358979323846;
int thickness = 1 + (rand() % 10);
HDC hdc;

struct Point2D { int x, y; };

Point2D getVertex(int cx, int cy, double r, int i) {
    double angle_rad = (60 * i) * PI / 180.0;
    return { (int)(cx + r * cos(angle_rad)), (int)(cy + r * sin(angle_rad)) };
}

void DrawHexRandom(int cx, int cy, double R) {    
    int styleIdx = rand() % 3; 
    int penStyle = PS_SOLID;
    if (styleIdx == 1) penStyle = PS_DASH;
    if (styleIdx == 2) penStyle = PS_DOT;


    COLORREF color = RGB(rand()%200, rand()%200, rand()%200);

    HPEN hPen = CreatePen(penStyle, thickness, color);
    HPEN hOldPen = (HPEN)SelectObject(hdc, hPen);

    bool fill = (rand() % 3 == 0); 
    HBRUSH hBrush = (HBRUSH)GetStockObject(NULL_BRUSH);
    if (fill) {
        int hatchStyle = rand() % 6; 
        COLORREF hatchColor = RGB(rand()%255, rand()%255, rand()%255);
        hBrush = CreateHatchBrush(hatchStyle, hatchColor);
    }
    HBRUSH hOldBrush = (HBRUSH)SelectObject(hdc, hBrush);

    // --- ГЕОМЕТРІЯ ---
    POINT pts[6];
    double r_inner = R * 0.5;
    POINT pts_inner[6];
    

    for (int i = 0; i < 6; i++) {
        Point2D p = getVertex(cx, cy, R, i);
        pts[i].x = p.x; pts[i].y = p.y;
        
        Point2D pi = getVertex(cx, cy, r_inner, i);
        pts_inner[i].x = pi.x; pts_inner[i].y = pi.y;
    }
    int innerThickness = max(1, thickness / 2);
    HPEN hInnerPen = CreatePen(PS_SOLID, innerThickness, RGB(80, 80, 80));
    Polygon(hdc, pts, 6);
    SelectObject(hdc, hInnerPen);
    for (int i = 0; i < 6; i++) {
        int j = (i + 5) % 6;

        MoveToEx(hdc, pts[i].x, pts[i].y, NULL);
        LineTo(hdc, pts_inner[j].x, pts_inner[j].y);

        MoveToEx(hdc, pts_inner[i].x, pts_inner[i].y, NULL);
        LineTo(hdc, pts_inner[(i+1)%6].x, pts_inner[(i+1)%6].y);

        MoveToEx(hdc, pts_inner[1].x, pts_inner[1].y, NULL);
        LineTo(hdc, pts_inner[4].x, pts_inner[4].y);
    }

    SelectObject(hdc, hOldPen);
    SelectObject(hdc, hOldBrush);
    DeleteObject(hPen);
    DeleteObject(hInnerPen);
    if (fill) DeleteObject(hBrush);
}

int main() {
    srand(time(0));
    HWND hwnd = GetConsoleWindow();
    hdc = GetDC(hwnd);

    system("color F0");
    system("cls");
    cout << "Lab 4: Randomized Console GDI Ornament" << endl;
    cout << "Features: Random colors, dashed lines, hatching, thickness." << endl;

    double R = 35.0 + (rand() % 50);
    
    int maxRows = 10 + (rand() % 20);
    int maxCols = 10 + (rand() % 20);

    double horiz = R * 1.5;      // горизонтальна відстань
    double vert  = R * sqrt(3);  // вертикальна відстань
    int startX = 120, startY = 200;

    for (int row = 0; row < maxRows; row++) {
        for (int col = 0; col < maxCols; col++) {
            int x = (int)(col * horiz) + startX;
            int y = (int)(row * vert) + startY;
            if (col % 2 == 1) y += vert / 2.0;   // зсув для «медового» порядку

            DrawHexRandom(x, y, R);
            Sleep(1);
        }
    }

    ReleaseDC(hwnd, hdc);

    cin.get();
    return 0;
}