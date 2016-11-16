using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public class TelescopeStructure : MonoBehaviour
    {
        public List<TelescopeJuncture> junctures;
        public List<TelescopeSegment> segments;
        
        /// <summary>
        /// Generates an OpenSCAD file, along with auxilliary .STL files,
        /// that can be compiled into 3D-printable geometry for this
        /// telescope structure.
        /// </summary>
        public void GenerateSCAD()
        {
            // Retract the structure completely
            foreach (TelescopeSegment segment in segments)
            {
                segment.ExtendImmediate(0);
            }

            // First output 
        }

        // Update is called once per frame
        void Update()
        {

        }
    }

}