using UnityEngine;
using System.Collections.Generic;
using UnityEditor;

namespace Telescopes.UI
{
    [CustomEditor(typeof(TelescopingSegment))]
    public class TelescopeInspector : Editor
    {
        List<bool> foldouts;

        void OnEnable()
        {
            TelescopingSegment ts = (TelescopingSegment)target;

            if (ts.parameters == null)
            {
                ts.parameters = new List<TelescopeParameters>();

                while (ts.parameters.Count > ts.initNumShells)
                {
                    ts.parameters.RemoveAt(ts.parameters.Count - 1);
                    foldouts.RemoveAt(ts.parameters.Count - 1);
                }
                while (ts.parameters.Count < ts.initNumShells)
                {
                    if (ts.parameters.Count == 0)
                        ts.parameters.Add(new TelescopeParameters(1, 0.5f, ts.wallThickness, 0, 0, 0));
                    else
                        ts.parameters.Add(new TelescopeParameters(0, 0, ts.wallThickness, 0, 0, 0));
                    foldouts.Add(false);
                }
            }
            if (foldouts == null)
            {
                foldouts = new List<bool>();

                foreach (TelescopeParameters tp in ts.parameters)
                {
                    foldouts.Add(true);
                }
            }
        }

        public override void OnInspectorGUI()
        {
            TelescopingSegment ts = (TelescopingSegment)target;

            ts.material = (Material)EditorGUILayout.ObjectField("Material", ts.material, typeof(Material), false);
            ts.fountainPrefab = (GameObject)EditorGUILayout.ObjectField("Fountain Prefab", ts.fountainPrefab, typeof(GameObject), false);

            ts.rootSegment = EditorGUILayout.Toggle("Root Segment", ts.rootSegment);
            ts.ReversedOption = EditorGUILayout.Toggle("Reversed", ts.ReversedOption);

            ts.parent = (TelescopeElement)EditorGUILayout.ObjectField("Parent telescope", ts.parent, typeof(TelescopeElement), true);
            ts.parentElementNumber = EditorGUILayout.IntField("Parent Element Number", ts.parentElementNumber);
            ts.offsetFromParent = EditorGUILayout.Vector3Field("Offset From Parent", ts.offsetFromParent);

            EditorGUILayout.Space();

            // Parameters for the entire telescope
            ts.initNumShells = EditorGUILayout.IntField("Number of Shells", ts.initNumShells);
            ts.initialDirection = EditorGUILayout.Vector3Field("Initial Direction", ts.initialDirection);
            ts.initialUp = EditorGUILayout.Vector3Field("Initial Up", ts.initialUp);
            ts.wallThickness = EditorGUILayout.FloatField("Wall Thickness", ts.wallThickness);

            // Keep parameter list synced with shell count
            while (ts.parameters.Count > ts.initNumShells)
            {
                ts.parameters.RemoveAt(ts.parameters.Count - 1);
                foldouts.RemoveAt(ts.parameters.Count - 1);
            }
            while (ts.parameters.Count < ts.initNumShells)
            {
                if (ts.parameters.Count == 0)
                    ts.parameters.Add(new TelescopeParameters(1, 0.5f, ts.wallThickness, 0, 0, 0));
                else
                    ts.parameters.Add(new TelescopeParameters(0, 0, ts.wallThickness, 0, 0, 0));
                foldouts.Add(false);
            }

            EditorGUILayout.Space();

            // Let each shell be edited separately
            for (int i = 0; i < ts.parameters.Count; i++)
            {
                ts.parameters[i].thickness = ts.wallThickness;
                // Display options for this parameter
                foldouts[i] = EditorGUILayout.Foldout(foldouts[i], "Shell " + i);
                EditorGUILayout.Space();
                if (foldouts[i])
                {
                    EditorGUI.indentLevel++;
                    if (i == 0)
                    {
                        ts.parameters[i].length = EditorGUILayout.FloatField("Initial length", ts.parameters[i].length);
                        ts.parameters[i].radius = EditorGUILayout.FloatField("Initial radius", ts.parameters[i].radius);
                        ts.parameters[i].curvature = EditorGUILayout.FloatField("Curvature", ts.parameters[i].curvature);
                        ts.parameters[i].twistFromParent = EditorGUILayout.FloatField("Twist angle", ts.parameters[i].twistFromParent);
                        ts.parameters[i].torsion = EditorGUILayout.FloatField("Torsion", ts.parameters[i].torsion);
                    }
                    else
                    {
                        ts.parameters[i].length = EditorGUILayout.FloatField("Length change", ts.parameters[i].length);
                        ts.parameters[i].radius = EditorGUILayout.FloatField("Radius change", ts.parameters[i].radius);
                        ts.parameters[i].curvature = EditorGUILayout.FloatField("Curvature change", ts.parameters[i].curvature);
                        ts.parameters[i].twistFromParent = EditorGUILayout.FloatField("Twist angle", ts.parameters[i].twistFromParent);
                        ts.parameters[i].torsion = EditorGUILayout.FloatField("Torsion change", ts.parameters[i].torsion);
                    }
                    EditorGUILayout.Space();
                    EditorGUI.indentLevel--;
                }
            }
        }
    }

}