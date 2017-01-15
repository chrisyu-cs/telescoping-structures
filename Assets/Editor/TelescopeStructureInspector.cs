using UnityEngine;
using System.Collections.Generic;
using UnityEditor;

namespace Telescopes.UI
{
    [CustomEditor(typeof(TelescopeStructure))]
    public class TelescopeStructureInspector : Editor
    {
        Material shellMaterial;

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            EditorGUILayout.Space();

            shellMaterial = (Material)EditorGUILayout.ObjectField("Shell material", shellMaterial, typeof(Material), false);
            if (GUILayout.Button("Apply material") && shellMaterial)
            {
                Debug.Log("Apply material " + shellMaterial);

                TelescopeStructure structure = target as TelescopeStructure;

                foreach (TelescopeSegment seg in structure.segments)
                {
                    foreach (TelescopeShell shell in seg.shells)
                    {
                        shell.setMaterial(shellMaterial);
                    }
                }
            }

        }

    }
}