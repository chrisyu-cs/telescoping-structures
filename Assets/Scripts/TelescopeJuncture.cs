using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public class TelescopeJuncture : TelescopeElement
    {
        public TelescopeElement parent;
        public int parentElementNum;
        public float offsetAlongParent;
        public Vector3 offsetFromParent;

        public Mesh customMesh;

        public bool keepLocalPositionOnStart = false;

        public TelescopeSegment parentSegment;
        public List<TelescopeSegment> childSegments;

        private MeshFilter mFilter;
        private MeshRenderer meshRenderer;

        public LineRenderer line;

        private JunctureType previousType;
        public JunctureType junctureType;

        float sphereRadius;
        public float Radius;

        public Vector3 WorldSpaceMin
        {
            get
            {
                if (junctureType == JunctureType.Sphere)
                {
                    return new Vector3(-sphereRadius, -sphereRadius, -sphereRadius) + transform.position;
                }
                else return meshRenderer.bounds.min;
            }
        }

        // Use this for initialization
        void Start()
        {
            if (parent)
            {
                TelescopeElement parentElement = parent.getChildElement(parentElementNum);
                transform.parent = parentElement.transform;

                if (!keepLocalPositionOnStart)
                {
                    transform.localPosition = parentElement.getAttachmentLocation(offsetAlongParent);
                    transform.localRotation = parentElement.getAttachmentRotation(offsetAlongParent);
                }
            }

            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.parent = transform;
            sphere.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            sphere.transform.localPosition = Vector3.zero;

            line = gameObject.AddComponent<LineRenderer>();
            line.useWorldSpace = false;
            line.material = DesignerController.instance.defaultLineMaterial;
            line.SetWidth(0.1f, 0.1f);

            List<Vector3> points = new List<Vector3>();
            int numNeighbors = 0;

            if (parentSegment)
            {
                points.Add(parentSegment.LastShell.transform.position - transform.position);
                points.Add(Vector3.zero);
                numNeighbors++;
            }

            foreach (TelescopeSegment seg in childSegments)
            {
                points.Add(seg.FirstShell.transform.position - transform.position);
                points.Add(Vector3.zero);
            }

            numNeighbors += childSegments.Count;

            line.SetVertexCount(points.Count);
            line.SetPositions(points.ToArray());

            // If this bulb is a dead end, make it a sphere.
            junctureType = (numNeighbors <= 1) ? JunctureType.Sphere : JunctureType.ConvexHull;
            previousType = JunctureType.None;

            if (!mFilter) mFilter = gameObject.AddComponent<MeshFilter>();
            if (!meshRenderer)
            {
                meshRenderer = gameObject.AddComponent<MeshRenderer>();
                meshRenderer.material = DesignerController.instance.defaultTelescopeMaterial;
            }
        }

        public void SetMaterial(Material m)
        {
            meshRenderer.material = m;
        }

        public override int numChildElements()
        {
            return 1;
        }

        public override TelescopeElement getChildElement(int i)
        {
            return this;
        }

        public void SetParent(TelescopeElement element, int parentElementNum, float attachmentPoint)
        {
            if (element == null)
            {
                transform.parent = null;
                return;
            }

            parent = element;
            TelescopeElement parentElement = element.getChildElement(parentElementNum);
            this.parentElementNum = parentElementNum;
            this.offsetAlongParent = attachmentPoint;
            this.transform.parent = parentElement.transform;
            this.transform.localPosition = parentElement.getAttachmentLocation(attachmentPoint);
            this.transform.localRotation = parentElement.getAttachmentRotation(attachmentPoint);
        }

        public void SetParentToSegmentEnd(TelescopeSegment segment)
        {
            parent = segment;
            parentElementNum = segment.shells.Count - 1;

            parentSegment = segment;
            //SetParent(segment, segment.shells.Count - 1, 1);
        }

        Vector3 contactDirection(TelescopeShell shell, bool reversed)
        {
            if (!reversed) return shell.transform.forward;
            else
            {
                Vector3 dirLocal = shell.getLocalRotationAlongPath(1) * Vector3.forward;
                Vector3 dir = shell.transform.rotation * dirLocal;
                dir *= -1;
                return dir;
            }
        }

        public List<Vector3> GetAdjacentRings()
        {
            List<Vector3> adjacentPoints = new List<Vector3>();
            
            Quaternion invRot = Quaternion.Inverse(transform.rotation);

            if (parent && parent is TelescopeSegment)
            {
                TelescopeSegment ts = parent as TelescopeSegment;
                List<Vector3> lastRing = ts.LastVertRing;
                foreach (Vector3 v in lastRing)
                {
                    adjacentPoints.Add(invRot * (v - transform.position));
                }
            }

            foreach (var child in childSegments)
            {
                List<Vector3> firstRing = child.FirstVertRing;
                foreach (Vector3 v in firstRing)
                {
                    adjacentPoints.Add(invRot * (v - transform.position));
                }
            }

            return adjacentPoints;
        }

        public void MakeConvexHull()
        {
            List<Vector3> adjacentPoints = GetAdjacentRings();
            
            adjacentPoints.Add(Vector3.zero);
            Mesh conv = UnityDDG.GeometryUtils.ConvexHull3D(adjacentPoints);
            mFilter.mesh = conv;
        }

        public void MakeSphere()
        {
            Mesh m = DesignerController.instance.SphereMesh;

            List<Vector3> adjacentPoints = GetAdjacentRings();

            float maxDistance = 0;
            
            foreach (Vector3 v in adjacentPoints)
            {
                maxDistance = Mathf.Max(maxDistance, v.magnitude);
            }

            Vector3[] vertices = m.vertices;
            // The default sphere has radius 0.5
            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i] = vertices[i] * 2 * maxDistance;
            }
            mFilter.mesh.vertices = vertices;
            mFilter.mesh.triangles = m.triangles;

            mFilter.mesh.RecalculateBounds();
            mFilter.mesh.RecalculateNormals();

            sphereRadius = maxDistance;
        }

        public override void ExtendImmediate(float t)
        {
            // Nothing; bulbs don't reconfigure.
        }

        public override Vector3 getAttachmentLocation(float t)
        {
            return Vector3.zero;
        }

        public override Quaternion getAttachmentRotation(float t)
        {
            return Quaternion.identity;
        }

        static int JunctureCount = 0;

        public static TelescopeJuncture CreateJuncture(Vector3 position, float initialRadius)
        {
            // Create the actual object for the juncture
            GameObject junctureObj = new GameObject();
            junctureObj.name = "bulb" + (JunctureCount++);
            // Add the juncture component and initialize its fields
            TelescopeJuncture juncture = junctureObj.AddComponent<TelescopeJuncture>();
            juncture.transform.position = position;
            juncture.childSegments = new List<TelescopeSegment>();

            juncture.Radius = initialRadius;

            return juncture;
        }

        // Write a set of STLs for a juncture, and an OpenSCAD file that
        // compiles them all into one fused part.
        public string WriteSTLOfJuncture(Vector3 minOffset, string objName)
        {
            List<string> unionSTLs = new List<string>();
            List<string> diffSTLs = new List<string>();
            string bulbSTL = "";
            
            // If there is a parent segment, write that
            if (parentSegment)
            {
                TelescopeShell last = parentSegment.LastShell;

                // Export them in world coordinates so that the full
                // object can just be assembled without moving pieces around
                STLWriter.VectorTransform f =
                    (v => last.transform.rotation * v + last.transform.position - minOffset);

                if (parentSegment.NumShells > 1 && parentSegment.Reversed)
                {
                    Mesh inner = last.GenerateInnerVolume(parentSegment.shells[parentSegment.NumShells - 2].getParameters(),
                        -Constants.WALL_THICKNESS, extraRings: Constants.CUTS_PER_CYLINDER / 2);

                    string parentVolumeSTL = objName + "-parentInner.stl";
                    STLWriter.WriteSTLOfMesh(inner, "scad/" + parentVolumeSTL, f);

                    diffSTLs.Add(parentVolumeSTL);
                }
            }

            // Write each child segment as well
            int childNum = 0;
            foreach (TelescopeSegment childSegment in childSegments)
            {
                TelescopeShell first = childSegment.FirstShell;
                
                STLWriter.VectorTransform f =
                    (v => first.transform.rotation * v + first.transform.position - minOffset);

                // Write a mesh for the inner volume contained in the shell,
                // so that it can be subtracted out from the juncture
                if (childSegment.NumShells > 1 && !childSegment.Reversed)
                {
                    Mesh inner = first.GenerateInnerVolume(childSegment.shells[1].getParameters(),
                        -Constants.WALL_THICKNESS, extraRings: Constants.CUTS_PER_CYLINDER / 2);

                    string childVolumeSTL = objName + "-childInner" + childNum + ".stl";
                    STLWriter.WriteSTLOfMesh(inner, "scad/" + childVolumeSTL, f);

                    diffSTLs.Add(childVolumeSTL);
                }
                childNum++;
            }

            // Write this piece
            if (junctureType != JunctureType.None)
            {
                STLWriter.VectorTransform f =
                    (v => transform.rotation * v + transform.position - minOffset);

                bulbSTL = objName + ".stl";
                STLWriter.WriteSTLOfMesh(mFilter.mesh, "scad/" + bulbSTL, f);
            }

            string scadFile = objName + ".scad";
            STLWriter.WriteSCADOfJunctureSTLs(bulbSTL, unionSTLs, diffSTLs, "scad/" + scadFile);
            return scadFile;
        }

        public void MakeMesh()
        {
            // Create the mesh for the new juncture type
            switch (junctureType)
            {
                case JunctureType.None:
                    break;
                case JunctureType.ConvexHull:
                    MakeConvexHull();
                    break;
                case JunctureType.Sphere:
                    MakeSphere();
                    break;
                case JunctureType.CustomMesh:
                    mFilter.mesh.vertices = customMesh.vertices;
                    mFilter.mesh.triangles = customMesh.triangles;
                    mFilter.mesh.uv = customMesh.uv;
                    mFilter.mesh.RecalculateBounds();
                    mFilter.mesh.RecalculateNormals();
                    break;
                default:
                    break;
            }
        }

        public void ExportJunctureToOBJ()
        {
            string objName = name + ".obj";
            OBJWriter.ExportToOBJ(mFilter.mesh, objName);
        }

        void Update()
        {
            if (Input.GetKeyDown("h"))
            {
                MakeConvexHull();
            }

            if (Input.GetKeyDown("]"))
            {
                bool done = CollisionIteration(0.5f);
                MakeMesh();
                Debug.Log("No more collisions = " + done);
            }

            if (Input.GetKeyDown("end"))
            {
                ExportJunctureToOBJ();
            }
            
            /*
            if (Input.GetKey("left shift") && Input.GetKeyDown("enter"))
            {
                Vector3 minPoint = meshRenderer.bounds.min;

                if (parentSegment)
                {
                    minPoint = TelescopeUtils.VectorMin(minPoint, parentSegment.LastShell.WorldSpaceBounds.min);
                }

                foreach (TelescopeSegment childSegment in childSegments)
                {
                    minPoint = TelescopeUtils.VectorMin(minPoint, childSegment.FirstShell.WorldSpaceBounds.min);
                }

                WriteSTLOfJuncture(minPoint, name);
            }*/

            if (previousType != junctureType)
            {
                // Clear mesh data for the old juncture type
                switch (previousType)
                {
                    case JunctureType.None:
                        break;
                    case JunctureType.ConvexHull:
                    case JunctureType.Sphere:
                    case JunctureType.CustomMesh:
                        mFilter.mesh.Clear();
                        break;
                    default:
                        break;
                }

                previousType = junctureType;

                MakeMesh();
            }
        }

        void RotateChildSegment(TelescopeSegment seg, Quaternion localRotation)
        {
            Vector3 localPos = localRotation * seg.transform.localPosition;
            seg.transform.localPosition = localPos;

            Quaternion localRot = localRotation * seg.shells[0].transform.localRotation;
            seg.shells[0].transform.localRotation = localRot;
        }

        public bool CollisionIteration(float delta)
        {
            bool hasCollision = false;

            if (parentSegment)
            {
                foreach (TelescopeSegment seg in childSegments)
                {
                    TelescopeShell parentShell = parentSegment.LastShell;
                    TelescopeShell childShell = seg.FirstShell;
                    float moveDistance;
                    Vector3 moveDirection;

                    if (parentShell.CollidesWith(childShell, out moveDistance, out moveDirection))
                    {
                        hasCollision = true;
                        childShell.transform.position += 0.5f * moveDistance * delta * moveDirection;
                        parentShell.transform.position -= 0.5f * moveDistance * delta * moveDirection;
                    }
                }
            }

            for (int i = 0; i < childSegments.Count; i++)
            {
                for (int j = i + 1; j < childSegments.Count; j++)
                {
                    TelescopeShell shell1 = childSegments[i].FirstShell;
                    TelescopeShell shell2 = childSegments[j].FirstShell;
                    float moveDistance;
                    Vector3 moveDirection;

                    if (shell1.CollidesWith(shell2, out moveDistance, out moveDirection))
                    {
                        hasCollision = true;
                        shell2.transform.position += 0.5f * moveDistance * delta * moveDirection;
                        shell1.transform.position -= 0.5f * moveDistance * delta * moveDirection;
                    }
                }
            }

            return !hasCollision;
        }

        bool CollisionIterationOld(float delta)
        {
            bool allOK = true;

            if (parentSegment)
            {
                Vector3 parentTangent = Quaternion.Inverse(transform.rotation) * parentSegment.WorldEndTangent();
                float parentRadius = parentSegment.shells[parentSegment.shells.Count - 1].radius;

                float parentTheta = Mathf.Acos(parentRadius / Radius);

                // Project all segments away from the parent segment
                foreach (TelescopeSegment seg in childSegments)
                {
                    float childRadius = seg.shells[0].radius;
                    Vector3 childTangent = seg.LocalContactTangent();
                    float childTheta = Mathf.Asin(childRadius / Radius);
                    float angleBetween = Mathf.Acos(Vector3.Dot(parentTangent, childTangent));

                    /*
                    Vector3 worldParent = transform.rotation * parentTangent;
                    Vector3 worldChild = transform.rotation * childTangent;

                    List<Vector3> parentPts = new List<Vector3>();
                    parentPts.Add(transform.position);
                    parentPts.Add(transform.position + 5f * worldParent);

                    List<Vector3> childPts = new List<Vector3>();
                    childPts.Add(transform.position);
                    childPts.Add(transform.position + 5f * worldChild);

                    TelescopeUtils.DisplayLine(parentPts);
                    TelescopeUtils.DisplayLine(childPts);
                    */

                    if (parentTheta + childTheta > angleBetween)
                    {
                        allOK = false;

                        Debug.Log("Angle to parent = " + angleBetween + ", individual angles "
                            + parentTheta + " + " + childTheta + " = " + (parentTheta + childTheta));

                        float angleDiff = parentTheta + childTheta - angleBetween;
                        angleDiff *= Mathf.Rad2Deg;

                        Vector3 normal = Vector3.Cross(parentTangent, childTangent);
                        Quaternion rot = Quaternion.AngleAxis(delta * angleDiff, normal);

                        RotateChildSegment(seg, rot);
                    }
                }
            }

            // Project all pairs of child segments
            for (int i = 0; i < childSegments.Count; i++)
            {
                for (int j = i + 1; j < childSegments.Count; j++)
                {
                    Vector3 tangent1 = childSegments[i].LocalContactTangent();
                    Vector3 tangent2 = childSegments[j].LocalContactTangent();

                    float radius1 = childSegments[i].shells[0].radius;
                    float radius2 = childSegments[j].shells[0].radius;

                    float angleBetween = Mathf.Acos(Vector3.Dot(tangent1, tangent2));
                    float theta1 = Mathf.Asin(radius1 / Radius);
                    float theta2 = Mathf.Asin(radius2 / Radius);

                    if (theta1 + theta2 > angleBetween)
                    {
                        allOK = false;

                        Debug.Log("Angle between = " + angleBetween + ", individual angles "
                            + theta1 + " + " + theta2 + " = " + (theta1 + theta2));

                        float angleDiff = theta1 + theta2 - angleBetween;
                        angleDiff *= Mathf.Rad2Deg;

                        Vector3 normal = Vector3.Cross(tangent1, tangent2);
                        Quaternion rot1 = Quaternion.AngleAxis(delta * -angleDiff / 2, normal);
                        Quaternion rot2 = Quaternion.AngleAxis(delta * angleDiff / 2, normal);

                        RotateChildSegment(childSegments[i], rot1);
                        RotateChildSegment(childSegments[j], rot2);
                    }
                }
            }

            return allOK;
        }

    }
}

public enum JunctureType
{
    None, ConvexHull, Sphere, CustomMesh
}