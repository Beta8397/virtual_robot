package org.firstinspires.ftc.robotcore.external.tfod;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Interface representing a detection.
 *
 * @author Vasu Agrawal
 * @author lizlooney@google.com (Liz Looney)
 */
public interface Recognition {
  /**
   * Returns the label of the detected object.
   */
  String getLabel();

  /**
   * Returns the confidence of the detection.
   */
  float getConfidence();

  /**
   * Returns the left coordinate of the rectangle bounding the detected object.
   */
  float getLeft();

  /**
   * Returns the right coordinate of the rectangle bounding the detected object.
   */
  float getRight();

  /**
   * Returns the top coordinate of the rectangle bounding the detected object.
   */
  float getTop();

  /**
   * Returns the bottom coordinate of the rectangle bounding the detected object.
   */
  float getBottom();

  /**
   * Returns the width of the rectangle bounding the detected object.
   */
  float getWidth();

  /**
   * Returns the height of the rectangle bounding the detected object.
   */
  float getHeight();

  /**
   * Returns the width of the entire image.
   */
  int getImageWidth();

  /**
   * Returns the height of the entire image.
   */
  int getImageHeight();

  /**
   * Returns an estimation of the horizontal angle to the detected object.
   */
  double estimateAngleToObject(AngleUnit angleUnit);
}