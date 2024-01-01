package android.graphics;


public final class Bitmap {
    public static Bitmap createBitmap(int width, int height, Bitmap.Config config) {
        return new Bitmap();
    }

    public static class Config {
        public static final Bitmap.Config RGB_565 = new Config();
    }
}
