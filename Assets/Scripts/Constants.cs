using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public static class Constants
    {
        public static bool PIN_ENDPOINTS = false;
        public static bool ADD_SUPPORT_CHANNELS = true;

        public const float MM_PER_UNIT = 40f;

        public const float MIN_SHELL_LENGTH = 0.15f;

        public const float WALL_THICKNESS = 0.08f;//0.1f;

        public const float DEFAULT_MIN_RADIUS = 2f * WALL_THICKNESS;

        public const float INITIAL_SPLINE_SIZE = 0.25f;

        public const float TAPER_SLOPE = 0.025f;//0.04f;//0.025f;
        public const int ARC_SAMPLES = 10;
        public const float COSMETIC_TAPER_RATIO = 0f;

        public const float SHELL_GAP = 0.00f;//0.01f;
        public const float INDENT_RATIO = 0.55f;

        public const float QP_LOWER_BOUND = -1000;
        public const float QP_UPPER_BOUND = 1000;

        public const float CURVATURE_SOLVE_THRESHOLD = 50;
        public const float CURVE_SEGMENT_THRESHOLD = 1;

        public const int CUTS_PER_CYLINDER = 20;
        public const int VERTS_PER_CIRCLE = 90;
        public const int OVERHANG_CUTS = 5;//5;

        public const int GROOVE_CUT_RADIUS = 3;
        public const int FIN_CUTS = 3;

        public const float COLLISION_GAP = 0.1f;

        static Constants()
        {
            /*
            if (FIN_CUTS >= OVERHANG_CUTS)
            {
                throw new System.Exception("Fins cannot be taller than the overhang amount.");
            }*/
        }
    }

}