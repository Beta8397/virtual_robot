package Autonomous;

public abstract class Shape {
    public int x, y;
    public int top, right, bottom, left;
    public int width, height;

    public Shape() {}
    public Shape(int x, int y) {
        this.x = x;
        this.y = y;
    }
    public Shape(int x, int y, int width, int height) {
        this(x, y);
        this.width = width;
        this.height = height;
        calculateSides();
    }
    public abstract double getArea();
    public abstract double getPerimeter();
    private void calculateSides() {
        top = y - (int)(height / 2.0 + 0.5);
        bottom = y + (int)(height / 2.0 + 0.5);
        left = x - (int)(width / 2.0 + 0.5);
        right = x + (int)(width / 2.0 + 0.5);
    }
}
