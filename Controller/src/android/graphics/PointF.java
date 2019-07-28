package android.graphics;

/**
 * PointF class to provide functionality similar to that found in android.graphics.PointF
 */
public class PointF {

    public float x = 0, y = 0;

    public PointF(){}

    public PointF(float x, float y){
        this.x = x;
        this.y = y;
    }

    public void set(float x, float y){
        this.x = x;
        this.y = y;
    }

    @Override
    public boolean equals(Object other){
        if (other == null) return false;
        if (this.getClass() != other.getClass()) return false;
        return this.x == ((PointF)other).x && this.y == ((PointF)other).y;
    }

    public boolean equals(float x, float y) {
        return this.x == x && this.y == y;
    }

    public float length() {
        return (float) Math.sqrt(x * x + y * y);
    }

    public static float length(float x, float y){
        return (float) Math.sqrt(x * x + y * y);
    }

    public void negate(){
        this.x = -this.x;
        this.y = -this.y;
    }

    public void offset(float dx, float dy){
        this.x += dx;
        this.y += dy;
    }


}
