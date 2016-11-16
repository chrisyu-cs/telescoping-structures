using UnityEngine;
using System.Collections.Generic;
using System;

namespace Telescopes
{
    [System.Serializable]
    public class TelescopeSegment : TelescopeElement
    {
        public Material material;

        public bool rootSegment = true;
        public bool ReversedOption;
        public bool Reversed;

        public GameObject fountainPrefab;

        [Tooltip("The direction of the first shell.")]
        public Vector3 initialDirection = Vector3.forward;
        public Vector3 initialUp = Vector3.up;
        [Tooltip("How many shells will be in the structure.")]
        public int initNumShells = 4;

        public int NumShells
        {
            get { return shells.Count; }
        }

        [Tooltip("The length of the first shell.")]
        public float initialLength = 1;
        [Tooltip("The radius of the first shell.")]
        public float initialRadius = 0.5f;
        [Tooltip("The amount of curvature of the structure.")]
        public float initialCurvature = 0f;
        [Tooltip("The angle that the curvature tends toward. 0 degrees = up.")]
        public float curvatureRotation = 0f;

        public float wallThickness
        {
            get
            {
                return Constants.DEFAULT_WALL_THICKNESS;
            }
        }

        public List<TelescopeParameters> parameters;
        public List<TelescopeParameters> concreteParameters;

        public List<TelescopeShell> shells;
        private GameObject telescopeRootShell;

        public TelescopeShell FirstShell
        {
            get
            {
                return shells[0];
            }
        }

        public TelescopeShell LastShell
        {
            get
            {
                return shells[shells.Count - 1];
            }
        }

        public List<Vector3> FirstVertRing
        {
            get
            {
                TelescopeShell baseShell = FirstShell; 
                List<Vector3> firstRing = (Reversed) ? baseShell.TopRing : baseShell.BottomRing;
                List<Vector3> worldSpace = new List<Vector3>();
                foreach (Vector3 v in firstRing)
                {
                    worldSpace.Add(baseShell.transform.rotation * v + baseShell.transform.position);
                }
                return worldSpace;
            }
        }

        public List<Vector3> LastVertRing
        {
            get
            {
                TelescopeShell endShell = LastShell;
                List<Vector3> lastRing = (!Reversed) ? endShell.TopRing : endShell.BottomRing;
                List<Vector3> worldSpace = new List<Vector3>();
                foreach (Vector3 v in lastRing)
                {
                    worldSpace.Add(endShell.transform.rotation * v + endShell.transform.position);
                }
                return worldSpace;
            }
        }

        public TelescopeElement parent;
        public int parentElementNumber;
        public Vector3 offsetFromParent;

        public bool keepLocalPositionOnStart = false;
        public SegmentParametersMode paramMode = SegmentParametersMode.Diffs;

        public TelescopeParameters DefaultChildDiff
        {
            get
            {
                TelescopeParameters tp = new TelescopeParameters(0, -wallThickness, wallThickness, 0, 0, 0);
                return tp;
            }
        }

        public void SetParent(TelescopeJuncture bulb)
        {
            parent = bulb;
            parentElementNumber = 0;
        }

        public override int numChildElements()
        {
            return shells.Count;
        }

        public override TelescopeElement getChildElement(int i)
        {
            return shells[i];
        }

        public TelescopeShell addChildShell(TelescopeShell parent,
            TelescopeParameters parentParams, TelescopeParameters childParams,
            TelescopeParameters nextParams, bool doOverhang)
        {
            int i = shells.Count;
            // Make the new shell, and set the previous shell as its parent
            GameObject shellObj = new GameObject();
            shellObj.transform.parent = parent.transform;
            shellObj.name = "shell" + i;

            // Make the geometry, etc.
            TelescopeShell newShell = shellObj.AddComponent<TelescopeShell>();
            newShell.GenerateGeometry(childParams, nextParams, overhang: doOverhang);
            newShell.setMaterial(material);

            // Set the shell's rest transformation relative to its parent.
            // When the shell's current extension ratio is 0, this is where
            // it is located relative to its parent.
            // newShell.baseRadians = newShell.radiansOfLength(wallThickness);
            newShell.baseTranslation = TelescopeUtils.childBasePosition(parentParams, childParams);
            newShell.baseRotation = TelescopeUtils.childBaseRotation(parentParams, childParams);
            shells.Add(newShell);

            shellObj.layer = 8;

            newShell.containingSegment = this;

            return newShell;
        }

        void DeleteTelescope()
        {
            if (telescopeRootShell)
            {
                shells.Clear();
                Destroy(telescopeRootShell);
            }
        }

        public List<TelescopeParameters> getParamList()
        {
            List<TelescopeParameters> allParams = new List<TelescopeParameters>();
            foreach (TelescopeShell ts in shells)
            {
                allParams.Add(ts.getParameters());
            }
            
            return allParams;
        }

        public override Vector3 getAttachmentLocation(float t)
        {
            return Vector3.zero;
        }

        public override Quaternion getAttachmentRotation(float t)
        {
            return Quaternion.identity;
        }

        public void MakeAllShells(List<TelescopeParameters> paramList, bool reversed = false)
        {
            Debug.Log("Wall = " + Constants.DEFAULT_WALL_THICKNESS + ", slope = " + Constants.TAPER_SLOPE +
                ", gap = " + Constants.SHELL_GAP + ", indent = " + Constants.INDENT_RATIO);

            DeleteTelescope();
            
            // Create an object for the first shell
            GameObject rootShellObj = new GameObject();
            rootShellObj.name = "shell0";
            rootShellObj.transform.parent = this.transform;
            rootShellObj.transform.localPosition = Vector3.zero;

            telescopeRootShell = rootShellObj;

            // Make the shell geometry
            TelescopeShell shell = rootShellObj.AddComponent<TelescopeShell>();
            // Get the twist impulse of the shell after, since we need to cut the
            // grooves in this shell to enable it
            TelescopeParameters params1 = (paramList.Count > 1) ? paramList[1] : null;
            shell.GenerateGeometry(paramList[0], params1, outerGroove: false);
            shell.setMaterial(material);

            Debug.Log("shell thickness " + shell.thickness + ", param thickness " + paramList[0].thickness);

            // Shells don't know anything about their position/rotation,
            // so we set that here.
            Quaternion initialFacing = Quaternion.LookRotation(initialDirection, initialUp);
            rootShellObj.transform.rotation = initialFacing;
            rootShellObj.layer = 8;

            shell.containingSegment = this;

            // shell.baseRadians = 0;
            shell.baseTranslation = Vector3.zero;
            shell.baseRotation = Quaternion.identity;

            shells.Add(shell);
            shell.isRoot = true;

            // Make all of the child shells here.
            TelescopeShell prevShell = shell;
            TelescopeParameters previousParams = paramList[0];
            TelescopeParameters currentParams = paramList[0];

            float accumulatedTaper = shell.getTaperLoss();

            for (int i = 1; i < paramList.Count; i++)
            {
                // Get the computed parameters for this and the previous shell.
                currentParams = paramList[i];
                previousParams = paramList[i - 1];

                // Shrink the radius by the accumulated taper so far.
                currentParams.radius -= accumulatedTaper;

                // Get the twist impulse for the next shell, so that we can cut the grooves.
                TelescopeParameters nextParams = (i < paramList.Count - 1) ? paramList[i + 1] : null;


                TelescopeParameters taperedParams = (nextParams != null) ? new TelescopeParameters(nextParams) : null;
                if (taperedParams != null) taperedParams.radius -= accumulatedTaper;

                // Add it.
                bool doOverhang = (i < paramList.Count - 1);
                prevShell = addChildShell(prevShell, previousParams, currentParams, taperedParams, doOverhang);
                accumulatedTaper += prevShell.getTaperLoss();
            }

            
            if (fountainPrefab)
            {
                GameObject fountain = Instantiate<GameObject>(fountainPrefab);
                fountain.transform.parent = prevShell.transform;
            }
        }

        public void MakeShellsFromDiffs(List<TelescopeParameters> diffs)
        {
            initNumShells = diffs.Count;
            parameters = diffs;
            shells = new List<TelescopeShell>();
            initialDirection.Normalize();

            // Compute the absolute parameter values from the list of diffs we are given.
            List<TelescopeParameters> concreteParams = new List<TelescopeParameters>();
            TelescopeParameters theParams = diffs[0];

            concreteParams.Add(new TelescopeParameters(diffs[0]));
            for (int i = 1; i < initNumShells; i++)
            {
                if (diffs[i].radius > -wallThickness) diffs[i].radius = -wallThickness;
                theParams = theParams + diffs[i];
                theParams.length -= 0; // wallThickness;
                //theParams.radius -= wallThickness;
                concreteParams.Add(theParams);
            }

            // Make sure the final shell is greater than the minimum possible width.
            if (concreteParams[concreteParams.Count - 1].radius < wallThickness)
            {
                concreteParams[concreteParams.Count - 1].radius = wallThickness;
            }

            // Make sure that all the shells fit inside each other.
            TelescopeUtils.growChainToFit(concreteParams);

            MakeShellsFromFinalList(concreteParams);
        }

        public void MakeShellsFromConcrete(List<TelescopeParameters> concreteParams)
        {
            initNumShells = concreteParams.Count;
            shells = new List<TelescopeShell>();
            initialDirection.Normalize();

            MakeShellsFromFinalList(concreteParams);
        }

        void MakeShellsFromFinalList(List<TelescopeParameters> concreteParams)
        {
            // Construct all of the shells from this parameter list.
            MakeAllShells(concreteParams);
            concreteParameters = concreteParams;
            currentExtension = 1;
            SetShellExtensions(currentExtension);
        }

        // Use this for initialization
        void Awake()
        {
            if (parameters != null && paramMode == SegmentParametersMode.Diffs)
            {
                MakeShellsFromDiffs(parameters);
            }
            else if (concreteParameters != null && paramMode == SegmentParametersMode.Concrete)
            {
                MakeShellsFromConcrete(parameters);
            }
        }

        void Start()
        {
            if (parent)
            {
                TelescopeElement element = parent.getChildElement(parentElementNumber);
                this.transform.parent = element.transform;
                if (keepLocalPositionOnStart)
                {
                    offsetFromParent = this.transform.position - element.transform.position;
                }
                else this.transform.position = element.transform.position + offsetFromParent;
            }
        }

        private float extensionTime = 0;
        private float extensionTimespan = 0;
        private float oldExtension = 0;
        private float currentExtension = 0;
        private float targetExtension = 0;
        private bool currentlyInterp = false;

        public void ExtendTo(float t)
        {
            ExtendTo(t, Mathf.Log(shells.Count));
        }

        void ExtendTo(float t, float overTime)
        {
            extensionTime = 0;
            extensionTimespan = overTime;
            oldExtension = currentExtension;
            targetExtension = t;
            currentlyInterp = true;
        }

        public void SetShellExtensions(float t)
        {
            float tRemaining = t;
            float tStep = 1f / (shells.Count - 1);

            shells[0].extensionRatio = Mathf.Clamp01(t);

            for (int i = 1; i < shells.Count; i++)
            {
                TelescopeShell ts = shells[i];
                if (tRemaining > tStep)
                {
                    ts.extensionRatio = 1;
                    tRemaining -= tStep;
                }
                else if (tRemaining > 0)
                {
                    float tNormalized = tRemaining / tStep;
                    ts.extensionRatio = tNormalized;
                    tRemaining = 0;
                }
                else
                {
                    ts.extensionRatio = 0;
                }
            }
        }

        public override void ExtendImmediate(float t)
        {
            SetShellExtensions(t);
            for (int i = 1; i < shells.Count; i++)
            {
                shells[i].SetTransform();
            }
        }

        /*
        public Vector3 WorldEndPosition()
        {
            TelescopingShell lastShell = shells[shells.Count - 1];
        }*/

        public Vector3 WorldEndTangent()
        {
            TelescopeShell lastShell = shells[shells.Count - 1];

            if (!Reversed)
            {
                Quaternion baseQ = lastShell.transform.rotation;
                Quaternion local = lastShell.getLocalRotationAlongPath(1);
                Quaternion combined = baseQ * local;

                Vector3 worldForward = combined * Vector3.forward;
                return -worldForward;
            }

            else
            {
                Quaternion local = lastShell.transform.rotation;
                return local * Vector3.forward;
            }
        }

        public Vector3 LocalContactTangent()
        {
            TelescopeShell firstShell = shells[0];
            if (!Reversed)
            {
                Quaternion local = firstShell.transform.localRotation;
                return local * Vector3.forward;
            }
            else
            {
                Quaternion baseQ = firstShell.transform.localRotation;
                Quaternion pathQ = firstShell.getLocalRotationAlongPath(1);

                Quaternion local = baseQ * pathQ;

                return -(local * Vector3.forward);
            }
        }

        public void ReverseTelescope()
        {
            Debug.Log("Reverse");
            Reversed = !Reversed;
            ReversedOption = !ReversedOption;
            // Extend the shell fully so that we have the correct world positions.
            ExtendImmediate(1);

            Transform parentOfRootShell = shells[0].transform.parent;

            // Unparent all shells.
            foreach (var shell in shells)
            {
                shell.transform.parent = parentOfRootShell;
            }

            // Reverse the parent order.
            for (int i = shells.Count - 2; i >= 0; i--)
            {
                shells[i].transform.parent = shells[i + 1].transform;
            }

            foreach (var shell in shells)
            {
                shell.Reversed = !shell.Reversed;
            }

            shells.Reverse();
            SetShellExtensions(currentExtension);
        }

        public void PrependShell()
        {
            Vector3 loc = shells[0].getLocalLocationAlongPath(-1);
            Quaternion rot = shells[0].getLocalRotationAlongPath(-1);

            TelescopeParameters p = new TelescopeParameters(shells[0].getParameters());
            p.radius += p.thickness;

            // Create the new shell object
            GameObject newShell = new GameObject();
            TelescopeShell shell = newShell.AddComponent<TelescopeShell>();
            shell.GenerateGeometry(p, null);
            shell.setMaterial(material);

            Vector3 shellLoc = shells[0].transform.position;
            Quaternion shellRot = shells[0].transform.rotation;

            // Move the new shell to be in position to receive the old one
            shell.transform.parent = shells[0].transform.parent;
            shell.transform.position = shellLoc + shellRot * loc;
            shell.transform.rotation = shellRot * rot;

            // Reparent the old starting shell so it becomes the child of the new one
            shells[0].baseTranslation = Vector3.zero;
            shells[0].baseRotation = Quaternion.identity;
            shells[0].transform.parent = shell.transform;
            shell.extensionRatio = shells[0].extensionRatio;
            shells.Insert(0, shell);
        }

        // Update is called once per frame
        void Update()
        {
            if (ReversedOption != Reversed)
            {
                ReverseTelescope();
                Reversed = ReversedOption;
            }

            
            /*
            if (Input.GetKey("left shift") && Input.GetKeyDown("enter"))
            {
                //STLWriter.WriteSTLOfSegment(this, name + ".stl");

                Mesh volume = shells[0].GenerateInnerVolume(shells[1].getParameters(), -0.5f);
                GameObject innerVolume = new GameObject();
                MeshFilter mf = innerVolume.AddComponent<MeshFilter>();
                innerVolume.AddComponent<MeshRenderer>().material = DesignerController.instance.defaultTelescopeMaterial;
                mf.mesh = volume;

                innerVolume.transform.parent = shells[0].transform;
                innerVolume.transform.localPosition = Vector3.zero;
                innerVolume.transform.localRotation = Quaternion.identity;

                //shells[0].ReplaceMesh(volume);

                for (int i = 1; i < shells.Count; i++)
                {
                    shells[i].GetComponent<MeshRenderer>().enabled = false;
                }

                //STLWriter.WriteSTLOfMesh(shells[0].mesh, "scad/innerVolume.stl");
            }*/

            if (rootSegment)
            {
                if (Input.GetKeyDown("e"))
                {
                    ExtendTo(1, Mathf.Log(shells.Count));
                }
                else if (Input.GetKeyDown("q"))
                {
                    ExtendTo(0, Mathf.Log(shells.Count));
                } 
            }

            if (currentlyInterp)
            {
                extensionTime += Time.deltaTime;
                currentExtension = Mathf.Lerp(oldExtension, targetExtension, extensionTime / extensionTimespan);
                if (extensionTime >= extensionTimespan)
                {
                    currentlyInterp = false;
                    currentExtension = targetExtension;
                }
                SetShellExtensions(currentExtension);
            }
            
            for (int i = 1; i < shells.Count; i++)
            {
                shells[i].SetTransform();
            }

            if (Input.GetKeyDown("/"))
            {
                List<Vector3> firstRing = FirstVertRing;
                List<Vector3> lastRing = LastVertRing;

                foreach (Vector3 v in firstRing)
                {
                    GameObject g = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    g.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
                    g.transform.position = v;
                }
                
                foreach (Vector3 v in lastRing)
                {
                    GameObject g = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    g.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
                    g.transform.position = v;
                }
            }
        }
    }

    [System.Serializable]
    public class TelescopeParameters
    {
        public float length;
        public float radius;
        public float thickness;
        public float curvature;
        public float torsion;
        public float twistFromParent;

        public override string ToString()
        {
            return "Length " + length + ", radius " + radius + ", thickness " + thickness
                + ", curvature " + curvature + ", twist " + twistFromParent; 
        }

        public TelescopeParameters(float length, float radius, float thickness,
            float curvature, float torsion, float twistImpulse)
        {
            this.length = length;
            this.radius = radius;
            this.thickness = thickness;
            this.curvature = curvature;
            this.torsion = torsion;
            this.twistFromParent = twistImpulse;
        }

        public static TelescopeParameters operator +(TelescopeParameters t1, TelescopeParameters t2)
        {
            TelescopeParameters sum = new TelescopeParameters(t1, t2);
            return sum;
        }

        public TelescopeParameters(TelescopeParameters toCopy)
        {
            length = toCopy.length;
            radius = toCopy.radius;
            thickness = toCopy.thickness;
            curvature = toCopy.curvature;
            torsion = toCopy.torsion;
            twistFromParent = toCopy.twistFromParent;
        }

        public TelescopeParameters(TelescopeParameters baseParams, TelescopeParameters diff)
        {
            length = baseParams.length + diff.length;
            radius = baseParams.radius + diff.radius;
            thickness = baseParams.thickness;
            curvature = baseParams.curvature + diff.curvature;
            torsion = baseParams.torsion + diff.torsion;
            twistFromParent = diff.twistFromParent;
        }
    }

    public enum SegmentParametersMode
    {
        Diffs, Concrete
    }
}