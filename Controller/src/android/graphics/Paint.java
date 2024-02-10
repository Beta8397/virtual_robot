package android.graphics;

public class Paint {
    public Paint(){

    }
    public Paint(Paint original){

    }
    public void setColor(int color){

    }

    public void setStrokeWidth(float width){

    }

    public enum Style {
        /**
         * Geometry and text drawn with this style will be filled, ignoring all
         * stroke-related settings in the paint.
         */
        FILL            (0),
        /**
         * Geometry and text drawn with this style will be stroked, respecting
         * the stroke-related fields on the paint.
         */
        STROKE          (1),
        /**
         * Geometry and text drawn with this style will be both filled and
         * stroked at the same time, respecting the stroke-related fields on
         * the paint. This mode can give unexpected results if the geometry
         * is oriented counter-clockwise. This restriction does not apply to
         * either FILL or STROKE.
         */
        FILL_AND_STROKE (2);

        Style(int nativeInt) {
            this.nativeInt = nativeInt;
        }
        final int nativeInt;
    }

    public void setStyle(Style style){

    }
}
