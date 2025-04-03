#ifndef LINE_H
#define LINE_H


class Line
{
public:
    Line();
    Line(double gradient, double y_intercept);
    Line(double ax, double ay, double bx, double by);
    void fromPoints(double ax, double ay, double bx, double by);
    void setGradient(double gradient);
    void setYIntercept(double y_intercept);
    bool pointAboveLine(double x, double y);
private:
    double gradient_;
    double y_intercept_;
};

#endif // LINE_H
