/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
Modified by FTC Team Beta 8397 for use in the Virtual Robot simulator.
 */
package org.firstinspires.ftc.robotcore.external.matrices;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.NonConst;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * An {@link OpenGLMatrix} is a 4x4 matrix commonly used as a transformation matrix for 3D
 * homogeneous coordinates. The data layout of an {@link OpenGLMatrix} is used heavily in the
 * <a href="https://www.opengl.org/">OpenGL</a> high performance graphics standard.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Homogeneous_coordinates">Homogenous coordinates</a>
 * @see <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
 */
public class OpenGLMatrix extends ColumnMajorMatrixF {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    float[] data;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public OpenGLMatrix() {
        super(4, 4);
        this.data = new float[4 * 4];
        this.data[0] = this.data[5] = this.data[10] = this.data[15] = 1.0f;
    }

    public OpenGLMatrix(float[] data) {
        super(4, 4);
        this.data = data;
        if (this.data.length != 4 * 4) throw dimensionsError();
    }


    /**
     * Constructs an OpenGL matrix whose values are initialized from the other matrix.
     * The other matrix must have dimensions at most 4x4.
     *
     * @param him the matrix from which to initialize our own data
     */
    public OpenGLMatrix(MatrixF him) {
        this();
        if (him.numRows > 4 || him.numCols > 4) throw him.dimensionsError();
        for (int i = 0; i < Math.min(4, him.numRows); i++) {
            for (int j = 0; j < Math.min(4, him.numCols); j++) {
                this.put(i, j, him.get(i, j));
            }
        }
    }

    @Override
    public MatrixF emptyMatrix(int numRows, int numCols) {
        if (numRows == 4 && numCols == 4)
            return new OpenGLMatrix();
        else
            return new GeneralMatrixF(numRows, numCols);
    }

    /**
     * Creates a matrix for rotation by the indicated angle around the indicated vector.
     */
    public static OpenGLMatrix rotation(AngleUnit angleUnit, float angle, float dx, float dy, float dz) {
        VectorF v = new VectorF(dx, dy, dz);
        v = v.normalized3D();
        VectorF x = new VectorF(1, 0, 0);
        VectorF m, n;
        if (Math.abs(v.dotProduct(x)) < 0.7) m = crossProduct(x, v);
        else m = crossProduct(new VectorF(0, 1, 0), v);
        m.normalized3D();
        n = crossProduct(v, m);
        float[] dataV = v.getData(), dataM = m.getData(), dataN = n.getData();
        float[] data = new float[16];
        angle = angleUnit.toRadians(angle);
        float cos = (float) Math.cos(angle);
        float sin = (float) Math.sin(angle);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                int k = 4*j + i;
                data[k] = dataV[i] * dataV[j] + (dataM[i] * dataM[j] + dataN[i] * dataN[j]) * cos
                        + (dataN[i] * dataM[j] - dataM[i] * dataN[j]) * sin;
            }
        data[15] = 1.0f;
        return new OpenGLMatrix(data);
    }

    private static VectorF crossProduct(VectorF v1, VectorF v2) {
        if (v1.length() != 3 || v2.length() != 3)
            throw new RuntimeException("Cross Product requires dimension of 3");
        float[] d1 = v1.getData();
        float[] d2 = v2.getData();
        return new VectorF(d1[1] * d2[2] - d1[2] * d2[1], d1[2] * d2[0] - d1[0] * d2[2], d1[0] * d2[1] - d1[1] * d2[0]);
    }

    /**
     * Creates a matrix for a rotation specified by three successive rotation angles.
     *
     * @see Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)
     */
    public static OpenGLMatrix rotation(AxesReference axesReference, AxesOrder axesOrder, AngleUnit angleUnit, float first, float second, float third) {
        OpenGLMatrix rotation = Orientation.getRotationMatrix(axesReference, axesOrder, angleUnit, first, second, third);
        return identityMatrix().multiplied(rotation);
    }

    public static OpenGLMatrix translation(float dx, float dy, float dz) {
        OpenGLMatrix result = new OpenGLMatrix();
        result.translate(dx, dy, dz);
        return result;
    }

    public static OpenGLMatrix identityMatrix() {
        return new OpenGLMatrix();
    }

    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    @Override
    public float[] getData() {
        return this.data;
    }

    //----------------------------------------------------------------------------------------------
    // Transformation matrix operations (in-place). These methods all return the receiver
    // in order to facilitate chaining.
    //
    // Note that these are some of the very view matrix operations that update-in-place rather than
    // returning a new matrix and leaving the receiver unmodified. Care must thus be taken to avoid
    // sharing the data of this matrix (using getData()) with other matrix-related objects and then
    // subsequently modifying this matrix.
    //----------------------------------------------------------------------------------------------

    @NonConst
    public void scale(float scaleX, float scaleY, float scaleZ) {
        OpenGLMatrix scaleMatrix = new OpenGLMatrix();
        float[] d = scaleMatrix.getData();
        d[0] = scaleX;  d[5] = scaleY;  d[10] = scaleZ;
        this.multiply(scaleMatrix);
    }

    @NonConst
    public void scale(float scale) {
        this.scale(scale, scale, scale);
    }

    @NonConst
    public void translate(float dx, float dy, float dz) {
        OpenGLMatrix translateMatrix = new OpenGLMatrix();
        float[] d = translateMatrix.getData();
        d[12] = dx;  d[13] = dy;  d[14] = dz;
        this.multiply(translateMatrix);
    }

    @NonConst
    public void rotate(AngleUnit angleUnit, float angle, float dx, float dy, float dz) {
        OpenGLMatrix rotationMatrix = OpenGLMatrix.rotation(angleUnit, angle, dx, dy, dz);
        this.multiply(rotationMatrix);
    }

    @NonConst
    public void rotate(AxesReference axesReference, AxesOrder axesOrder, AngleUnit angleUnit, float first, float second, float third) {
        OpenGLMatrix rotation = Orientation.getRotationMatrix(axesReference, axesOrder, angleUnit, first, second, third);
        this.data = this.multiplied(rotation).getData();
    }

    //----------------------------------------------------------------------------------------------
    // Transformation matrix operations
    //----------------------------------------------------------------------------------------------

    @Const
    public OpenGLMatrix scaled(float scaleX, float scaleY, float scaleZ) {
        OpenGLMatrix scaleMatrix = new OpenGLMatrix();
        float[] d = scaleMatrix.getData();
        d[0] = scaleX;  d[5] = scaleY;  d[10] = scaleZ;
        return this.multiplied(scaleMatrix);
    }

    @Const
    public OpenGLMatrix scaled(float scale) {
        return scaled(scale, scale, scale);
    }

    @Const
    public OpenGLMatrix translated(float dx, float dy, float dz) {
        OpenGLMatrix translateMatrix = new OpenGLMatrix();
        float[] d = translateMatrix.getData();
        d[12] = dx;  d[13] = dy;  d[14] = dz;
        return this.multiplied(translateMatrix);
    }

    @Const
    public OpenGLMatrix rotated(AngleUnit angleUnit, float angle, float dx, float dy, float dz) {
        OpenGLMatrix rotationMatrix = OpenGLMatrix.rotation(angleUnit, angle, dx, dy, dz);
        return this.multiplied(rotationMatrix);
    }

    @Const
    public OpenGLMatrix rotated(AxesReference axesReference, AxesOrder axesOrder, AngleUnit angleUnit, float first, float second, float third) {
        OpenGLMatrix rotation = Orientation.getRotationMatrix(axesReference, axesOrder, angleUnit, first, second, third);
        return this.multiplied(rotation);
    }

    //----------------------------------------------------------------------------------------------
    // Matrix operations
    //----------------------------------------------------------------------------------------------

    @Override
    @Const
    public OpenGLMatrix inverted() {
        MatrixF invertedMatrixF = super.inverted();
        float[] d = new float[16];
        for (int k=0; k<16; k++) d[k] = invertedMatrixF.get(k%4, k/4 );
        return new OpenGLMatrix(d);
    }

    @Override
    @Const
    public OpenGLMatrix transposed() {
        return (OpenGLMatrix) super.transposed();
    }

    @Const
    public OpenGLMatrix multiplied(OpenGLMatrix him) {
        OpenGLMatrix result = new OpenGLMatrix();
        int i, j;
        for (int k = 0; k < 16; k++) {
            j = k / 4;
            i = k % 4;
            result.data[k] = 0f;
            for (int m = 0; m < 4; m++) {
                result.data[k] += this.data[i + 4 * m] * him.data[m + 4 * j];
            }
        }
        return result;
    }

    @Override
    @Const
    public MatrixF multiplied(MatrixF him) {
        if (him instanceof OpenGLMatrix) {
            return this.multiplied((OpenGLMatrix) him);
        } else
            return super.multiplied(him);
    }

    /**
     * Updates the receiver to be the product of itself and another matrix.
     *
     * @param him the matrix with which the receiver is to be multiplied.
     */
    @NonConst
    public void multiply(OpenGLMatrix him) {
        this.data = this.multiplied(him).getData();
    }

    /**
     * Updates the receiver to be the product of itself and another matrix.
     *
     * @param him the matrix with which the receiver is to be multiplied.
     */
    @Override
    @NonConst
    public void multiply(MatrixF him) {
        if (him instanceof OpenGLMatrix) {
            this.multiply((OpenGLMatrix) him);
        } else
            super.multiply(him);
    }
}
