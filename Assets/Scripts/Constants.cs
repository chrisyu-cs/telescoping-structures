using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public static class Constants
    {
        public const float DEFAULT_WALL_THICKNESS = 0.05f;
        public const float INITIAL_SPLINE_SIZE = 0.25f;

        public const float TAPER_SLOPE = 0f;
        public const int ARC_SAMPLES = 10;
        public const float COSMETIC_TAPER_RATIO = 0f;

        public const float SHELL_GAP = 0.01f;

        public const float QP_LOWER_BOUND = -1000;
        public const float QP_UPPER_BOUND = 1000;

        public const float CURVATURE_SOLVE_THRESHOLD = 50;
        public const float CURVE_SEGMENT_THRESHOLD = 1;

        public const int CUTS_PER_CYLINDER = 20;
        public const int VERTS_PER_CIRCLE = 40;
        public const int OVERHANG_CUTS = 4;
    }

}