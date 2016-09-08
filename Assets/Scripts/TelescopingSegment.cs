using UnityEngine;
using System.Collections.Generic;
using System;

namespace Telescopes
{
    [System.Serializable]
    public class TelescopingSegment : TelescopeElement
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

        [Tooltip("How thick the walls of the geometry are to be.")]
        public float wallThickness = Constants.DEFAULT_WALL_THICKNESS;

        public List<TelescopeParameters> parameters;
        public List<TelescopeParameters> concreteParameters;

        public List<TelescopingShell> shells;
        private GameObject telescopeRootShell;

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

        public void SetParent(TelescopeBulb bulb)
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

        public TelescopingShell addChildShell(TelescopingShell parent,
            TelescopeParameters parentParams, TelescopeParameters childParams)
        {
            int i = shells.Count;
            // Make the new shell, and set the previous shell as its parent
            GameObject shellObj = new GameObject();
            shellObj.transform.parent = parent.transform;
            shellObj.name = "shell" + i;

            // Make the geometry, etc.
            TelescopingShell newShell = shellObj.AddComponent<TelescopingShell>();
            newShell.GenerateGeometry(childParams);
            newShell.setMaterial(material);

            // Set the shell's rest transformation relative to its parent.
            // When the shell's current extension ratio is 0, this is where
            // it is located relative to its parent.
            // newShell.baseRadians = newShell.radiansOfLength(wallThickness);
            newShell.baseTranslation = TelescopeUtils.childBasePosition(parentParams, childParams);
            newShell.baseRotation = TelescopeUtils.childBaseRotation(parentParams, childParams);
            shells.Add(newShell);

            CapsuleCollider cc = shellObj.AddComponent<CapsuleCollider>();
            cc.direction = 2;
            shellObj.layer = 8;
            Rigidbody rb = shellObj.AddComponent<Rigidbody>();
            rb.isKinematic = true;

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
            foreach (TelescopingShell ts in shells)
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
            DeleteTelescope();
            
            // Create an object for the first shell
            GameObject rootShellObj = new GameObject();
            rootShellObj.name = "shell0";
            rootShellObj.transform.parent = this.transform;
            rootShellObj.transform.localPosition = Vector3.zero;

            telescopeRootShell = rootShellObj;

            // Make the shell geometry
            TelescopingShell shell = rootShellObj.AddComponent<TelescopingShell>();
            shell.GenerateGeometry(paramList[0]);
            shell.setMaterial(material);

            // Shells don't know anything about their position/rotation,
            // so we set that here.
            Quaternion initialFacing = Quaternion.LookRotation(initialDirection, initialUp);
            rootShellObj.transform.rotation = initialFacing;
            CapsuleCollider cc = rootShellObj.AddComponent<CapsuleCollider>();
            cc.direction = 2;
            rootShellObj.layer = 8;
            Rigidbody rb = rootShellObj.AddComponent<Rigidbody>();
            rb.isKinematic = true;

            shell.containingSegment = this;

            // shell.baseRadians = 0;
            shell.baseTranslation = Vector3.zero;
            shell.baseRotation = Quaternion.identity;

            shells.Add(shell);
            shell.isRoot = true;

            // Make all of the child shells here.
            TelescopingShell prevShell = shell;
            TelescopeParameters previousParams = paramList[0];
            TelescopeParameters currentParams = paramList[0];

            float accumulatedTaper = shell.getTaperLoss();
            Debug.Log("taper loss = " + shell.getTaperLoss());

            for (int i = 1; i < paramList.Count; i++)
            {
                // Get the computed parameters for this and the previous shell.
                currentParams = paramList[i];
                previousParams = paramList[i - 1];

                currentParams.radius -= accumulatedTaper;

                // Add it.
                prevShell = addChildShell(prevShell, previousParams, currentParams);
                accumulatedTaper += prevShell.getTaperLoss();
                Debug.Log("taper loss = " + prevShell.getTaperLoss());
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
            shells = new List<TelescopingShell>();
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
            Debug.Log("num shells = " + initNumShells);
            shells = new List<TelescopingShell>();
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
            for (int i = 1; i < shells.Count; i++)
            {
                TelescopingShell ts = shells[i];
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

        // Update is called once per frame
        void Update()
        {
            if (ReversedOption != Reversed)
            {
                ReverseTelescope();
                Reversed = ReversedOption;
            }

            if (Input.GetKey("left shift") && Input.GetKeyDown("enter"))
            {
                STLWriter.WriteSTLOfSegment(this, name + ".stl");
            }

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

            // Live-update the orientation for better testing
            /*
            initialDirection.Normalize();
            Quaternion rotation = Quaternion.FromToRotation(Vector3.forward, initialDirection);
            Quaternion roll = Quaternion.AngleAxis(curvatureRotation, initialDirection);
            shells[0].transform.rotation = roll * rotation;*/
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