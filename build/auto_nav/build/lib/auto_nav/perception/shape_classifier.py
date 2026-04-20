"""
shape_classifier.py — classify a contour into a geometric shape.

Standalone utility function ``classify_contour`` can be imported by
other perception modules without starting a ROS node.

Shape decision tree
-------------------
  1. Circularity  = 4π·area / perimeter²
     if circularity > 0.78  → circle

  2. approxPolyDP vertices:
     3 vertices              → triangle
     4 vertices:
       aspect ratio ≈ 1.0   → square
       else                  → rectangle
     5 vertices              → pentagon
     6 vertices              → hexagon (treated as circle if circularity > 0.65)
     else (>6)               → circle (or unknown if circularity low)

  3. If still ambiguous      → unknown

Supported shapes:
  triangle, square, rectangle, circle, pentagon, hexagon, unknown
"""

from __future__ import annotations

import math

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Pure utility — no ROS dependency
# ---------------------------------------------------------------------------

def classify_contour(contour: np.ndarray) -> str:
    """
    Classify a single OpenCV contour into a named shape.

    Parameters
    ----------
    contour:
        Output of cv2.findContours (a single contour array).

    Returns
    -------
    str
        One of: 'triangle', 'square', 'rectangle', 'circle',
                'pentagon', 'hexagon', 'unknown'
    """
    area = cv2.contourArea(contour)
    if area < 1.0:
        return "unknown"

    perimeter = cv2.arcLength(contour, closed=True)
    if perimeter < 1.0:
        return "unknown"

    circularity = (4.0 * math.pi * area) / (perimeter * perimeter)

    # Approximate polygon
    epsilon  = 0.04 * perimeter
    approx   = cv2.approxPolyDP(contour, epsilon, closed=True)
    vertices = len(approx)

    # Check discrete vertex counts first (3–5) before applying circularity,
    # because a perfect square has circularity ≈ 0.785 which would otherwise
    # be mistaken for a circle.
    if vertices == 3:
        return "triangle"

    if vertices == 4:
        x, y, w, h = cv2.boundingRect(approx)
        ar = w / h if h > 0 else 1.0
        return "square" if 0.85 <= ar <= 1.15 else "rectangle"

    if vertices == 5:
        return "pentagon"

    # For 6+ vertices, use circularity to distinguish circle from hexagon / blob
    if circularity > 0.78:
        return "circle"

    if vertices == 6:
        return "circle" if circularity > 0.65 else "hexagon"

    if vertices > 6 and circularity > 0.60:
        return "circle"

    return "unknown"


# ---------------------------------------------------------------------------
# Optional: thin ROS 2 node wrapper that exposes a service / topic
# (not required for Step 4, but keeps architecture consistent)
# ---------------------------------------------------------------------------

def main(args=None) -> None:  # pragma: no cover
    """Entry point stub — ShapeClassifier is a library, not a standalone node."""
    print("shape_classifier is a library module.  "
          "Import classify_contour from auto_nav.perception.shape_classifier.")


if __name__ == "__main__":
    main()
