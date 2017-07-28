using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public static class Constants
    {
        /// <summary>
        /// Whether or not endpoints of curves should be pinned during
        /// curve optimization, allowing arc length to change instead.
        /// </summary>
        public static bool PIN_ENDPOINTS = false;

        /// <summary>
        /// Whether or not additional channels should be cut from geometry to
        /// help drain out dissolvable support material.
        /// </summary>
        public static bool ADD_SUPPORT_CHANNELS = true;

        /// <summary>
        /// Minimum allowed length of shells.
        /// </summary>
        public const float MIN_SHELL_LENGTH = 0.15f;

        /// <summary>
        /// Default thickness of shell walls.
        /// </summary>
        public const float WALL_THICKNESS = 0.08f;//0.1f;

        /// <summary>
        /// Minimum allowed radius of shells.
        /// </summary>
        public const float DEFAULT_MIN_RADIUS = 2f * WALL_THICKNESS;

        /// <summary>
        /// The rate at which shells naturally narrow.
        /// </summary>
        public const float TAPER_SLOPE = 0.025f;//0.04f;//0.025f;
        /// <summary>
        /// Narrowing of wall thickness. Easily creates invalid geometry, so best left at 0.
        /// </summary>
        public const float COSMETIC_TAPER_RATIO = 0f;

        /// <summary>
        /// Additional gap between shells, in order to create some leeway
        /// for printer tolerance and torsional impulse rotation.
        /// </summary>
        public const float SHELL_GAP = 0.01f;
        
        /// <summary>
        /// How deep (as a fraction of wall thickness) the guiding grooves along
        /// the side should be.
        /// </summary>
        public const float INDENT_RATIO = 0.55f;

        /// <summary>
        /// Bounds for Gurobi on the min and max possible integrated torsion.
        /// 1000 radians should be enough for most structures.
        /// </summary>
        public const float QP_LOWER_BOUND = -1000;
        public const float QP_UPPER_BOUND = 1000;

        /// <summary>
        /// These are unused. They were for experimental curvature segmentation,
        /// which we ended up scrapping.
        /// </summary>
        public const float CURVATURE_SOLVE_THRESHOLD = 50;
        public const float CURVE_SEGMENT_THRESHOLD = 1;

        /// <summary>
        /// Number of vertex circles along the axial direction of each shell.
        /// </summary>
        public const int CUTS_PER_CYLINDER = 40;
        /// <summary>
        /// Number of vertices per circle.
        /// </summary>
        public const int VERTS_PER_CIRCLE = 40;
        /// <summary>
        /// Number of extra cuts at the end of each shell -- there needs
        /// to be a bit of a buffer, to prevent shells from exiting their parents.
        /// </summary>
        public const int OVERHANG_CUTS = 5;//5;

        /// <summary>
        /// How wide, in vertices, the guiding grooves on the side should be.
        /// </summary>
        public const int GROOVE_CUT_RADIUS = 3;
        /// <summary>
        /// How tall, in vertices, the guiding "fins" at the base of each shell should be.
        /// </summary>
        public const int FIN_CUTS = 3;

        /// <summary>
        /// Small buffer distance that we use to determine when 2 shells are in collision.
        /// </summary>
        public const float COLLISION_GAP = 0.1f;

        static Constants()
        {
            if (FIN_CUTS >= OVERHANG_CUTS)
            {
                throw new System.Exception("Fins cannot be taller than the overhang amount.");
            }
        }
    }

}