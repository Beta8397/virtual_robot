package virtual_robot.util;

public class Vector2D {

    public double x, y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2D added(Vector2D v) {
        return new Vector2D(this.x + v.x, this.y + v.y);
    }

    public void add(Vector2D v){
        this.x += v.x;
        this.y += v.y;
    }

    public Vector2D subtracted(Vector2D v) {
        return new Vector2D(this.x - v.x, this.y - v.y);
    }

    public void subtract(Vector2D v){
        this.x -= v.x;
        this.y -= v.y;
    }

    public Vector2D multiplied(double s){
        return new Vector2D(s * this.x, s*this.y);
    }

    public void multiply(double s){
        this.x *= s;
        this.y *= s;
    }

    public Vector2D divided(double s){
        return new Vector2D(this.x / s, this.y / s);
    }

    public void divide(double s){
        this.x /= s;
        this.y /= s;
    }

    public double length(){
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public Vector2D normalized(){
        double length = this.length();
        if (length > 0) return this.divided(length);
        else {
            System.out.println("Vector2D normalized failed because of zero length");
            throw new RuntimeException();
        }
    }

    public void normalize(){
        double length = this.length();
        if (length > 0) {
            x /= length;
            y /= length;
        } else {
            System.out.println("Vector2D normalize failed because of zero length");
            throw new RuntimeException();
        }
    }

    public double dot(Vector2D v){
        return this.x * v.x + this.y * v.y;
    }

    public double cross(Vector2D v){
        return this.x * v.y - this.y * v.x;
    }

    public Vector2D rotated(double radians){
        double cos = Math.cos(radians);
        double sin = Math.sin(radians);
        return new Vector2D(x*cos - y*sin, x*sin + y*cos);
    }

    public void rotate(double radians) {
        double x0 = x;
        double y0 = y;
        double cos = Math.cos(radians);
        double sin = Math.sin(radians);
        x = x0 * cos - y0 * sin;
        y = x0 * sin + y0 * cos;
    }

}
