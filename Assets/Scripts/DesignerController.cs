using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

namespace Telescopes
{
    public class DesignerController : MonoBehaviour
    {
        public static DesignerController instance;
        public DesignerMode currentMode;
        public Text modeText;
        private TelescopingShell selected;

        public InputField lengthField;
        public InputField radiusField;
        public InputField curvatureField;
        public InputField twistField;

        public InputField splinePoints;
        public InputField impulsePoints;

        public Material defaultTelescopeMaterial;
        public Material selectedTelescopeMaterial;

        public InputCurve curve;

        public GameObject projectilePrefab;

        private SplineControlPoint selectedControlPt;
        private float selectedDepth;
        
        private TreeControlPoint selectedTreePt;
        private TreeControlPoint highlightedTreePt;
        private Transform selectedDraggable;

        public LayerMask normalMask;
        public LayerMask curveModeMask;
        public LayerMask curvatureModeMask;

        public InputField filenameField;

        private float shootTime = 0f;
        private float shootDelay = 0.05f;

        private Vector3 lastMousePos;

        public Toggle sparseToggle;
        public bool UseSparseSolve = true;

        public int numImpulses = 10;

        void Awake()
        {
            selectedDepth = 0;
            if (instance == null)
            {
                instance = this;
            }
            else if (instance != this)
            {
                Destroy(gameObject);
            }
        }

        // Use this for initialization
        void Start()
        {
            currentMode = DesignerMode.Shell;
            modeText.text = "Shell mode";
            splinePoints.text = "0";
            impulsePoints.text = numImpulses.ToString();
        }

        void RaycastShells(Vector3 clickPos)
        {
            Ray mouseRay = Camera.main.ScreenPointToRay(clickPos);
            RaycastHit hitInfo = new RaycastHit();

            LayerMask mask;

            switch (currentMode)
            {
                case DesignerMode.Shell:
                    mask = normalMask;
                    break;
                case DesignerMode.Curve:
                    mask = curveModeMask;
                    break;
                case DesignerMode.Curvature:
                    mask = curvatureModeMask;
                    break;
                default:
                    mask = normalMask;
                    break;
            }

            if (Physics.Raycast(mouseRay, out hitInfo, 20f, mask))
            {
                TelescopingShell selection = hitInfo.collider.GetComponent<TelescopingShell>();

                SplineControlPoint controlPt = hitInfo.collider.GetComponent<SplineControlPoint>();
                TreeControlPoint treePt = hitInfo.collider.GetComponent<TreeControlPoint>();
                CurvatureControlPoint curvPt = hitInfo.collider.GetComponent<CurvatureControlPoint>();

                if (selection) SelectShell(selection);

                else if (treePt)
                {
                    highlightedTreePt = treePt;
                    treePt.containingTree.SelectNode(treePt);
                    selectedTreePt = treePt;
                    selectedDepth = Camera.main.WorldToScreenPoint(treePt.transform.position).z;
                }
                else if (controlPt)
                {
                    selectedControlPt = controlPt;
                    selectedDepth = Camera.main.WorldToScreenPoint(controlPt.transform.position).z;
                }
                else if (curvPt)
                {
                    selectedDraggable = curvPt.transform;
                    selectedDepth = Camera.main.WorldToScreenPoint(curvPt.transform.position).z;
                }
            }
        }

        public void ToggleSparse()
        {
            UseSparseSolve = sparseToggle.isOn;
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetButtonDown("ToggleMode"))
            {
                if (currentMode == DesignerMode.Shell)
                {
                    currentMode = DesignerMode.Curve;
                    modeText.text = "Curve mode";
                }
                else if (currentMode == DesignerMode.Curve)
                {
                    currentMode = DesignerMode.Curvature;
                    modeText.text = "Curvature mode";
                }
                else if (currentMode == DesignerMode.Curvature)
                {
                    currentMode = DesignerMode.Shell;
                    modeText.text = "Shell mode";
                }
            }

            if (Input.GetMouseButtonDown(0))
            {
                Vector3 clickPos = Input.mousePosition;
                RaycastShells(clickPos);
            }

            else if (Input.GetMouseButton(0) && selectedControlPt)
            {
                Vector3 clickPos = Input.mousePosition;
                clickPos.z = selectedDepth;
                Vector3 worldPos = Camera.main.ScreenToWorldPoint(clickPos);
                selectedControlPt.Move(worldPos);
            }

            else if (Input.GetMouseButton(0) && selectedTreePt)
            {
                Vector3 clickPos = Input.mousePosition;
                clickPos.z = selectedDepth;
                Vector3 worldPos = Camera.main.ScreenToWorldPoint(clickPos);
                selectedTreePt.Move(worldPos);
            }

            else if (Input.GetMouseButton(0) && selectedDraggable)
            {
                Vector3 clickPos = Input.mousePosition;
                clickPos.z = selectedDepth;
                Vector3 worldPos = Camera.main.ScreenToWorldPoint(clickPos);
                selectedDraggable.transform.position = worldPos;
            }

            else if (Input.GetMouseButtonDown(2))
            {
                Vector3 clickPos = Input.mousePosition;
                RaycastShells(clickPos);
                // Store the eye-space position of the click.
                // Use eye space because we always want moving up/down to correspond
                // to bigger or smaller, regardless of camera orientation.
                lastMousePos = clickPos;
                lastMousePos.z = selectedDepth;
                lastMousePos = Camera.main.ScreenToViewportPoint(lastMousePos);
            }

            else if (Input.GetMouseButton(2) && selectedControlPt)
            {
                // Compute the new eye-space position the mouse has moved to
                Vector3 mousePos = Input.mousePosition;
                mousePos.z = selectedDepth;
                mousePos = Camera.main.ScreenToViewportPoint(mousePos);
                // Take difference in height
                float heightDiff = mousePos.y - lastMousePos.y;
                // Resize it
                selectedControlPt.ResizeDiff(heightDiff);
                // Update previous point to current
                lastMousePos = mousePos;
            }

            else if (Input.GetMouseButton(2) && selectedTreePt)
            {
                // Compute the new eye-space position the mouse has moved to
                Vector3 mousePos = Input.mousePosition;
                mousePos.z = selectedDepth;
                mousePos = Camera.main.ScreenToViewportPoint(mousePos);
                // Take difference in height
                float heightDiff = mousePos.y - lastMousePos.y;
                // Resize it
                selectedTreePt.ResizeDiff(heightDiff);
                // Update previous point to current
                lastMousePos = mousePos;
            }

            else if (Input.GetMouseButtonUp(0) || Input.GetMouseButtonUp(2))
            {
                selectedControlPt = null;
                selectedTreePt = null;
                selectedDraggable = null;
            }

            else if (Input.GetButtonDown("Cancel"))
            {
                Deselect();
            }

            else if (Input.GetButtonDown("Submit"))
            {
                if (highlightedTreePt)
                {
                    highlightedTreePt.containingTree.MakeTelescopes();
                }
            }

            shootTime += Time.deltaTime;
            if (shootTime > shootDelay && Input.GetKey("z"))
            {
                ShootSphere();
            }
        }

        public void SetSplinePoints()
        {
            if (!curve || !curve.isActiveAndEnabled) return;
            int num = int.Parse(splinePoints.text);
            if (num < 0) return;
            int currentNum = curve.points.Count;

            // Add more points until we have enough
            if (num > currentNum)
            {
                int numToAdd = num - currentNum;
                for (int i = 0; i < numToAdd; i++)
                {
                    curve.points.Add(new SphereNode(Constants.INITIAL_SPLINE_SIZE, Vector3.zero));
                }
            }
            else if (num < currentNum)
            {
                int numToRemove = currentNum - num;
                for (int i = 0; i < numToRemove; i++)
                {
                    curve.points.RemoveAt(curve.points.Count - 1);
                }
            }
        }

        public void Deselect()
        {
            if (selected) selected.setMaterial(defaultTelescopeMaterial);
            selected = null;
        }

        public void SelectShell(TelescopingShell selection)
        {
            if (!selection) return;
            if (selected)
            {
                selected.setMaterial(defaultTelescopeMaterial);
            }
            selected = selection;
            selected.setMaterial(selectedTelescopeMaterial);

            TelescopeParameters tp = selected.getParameters();
            lengthField.text = tp.length.ToString();
            radiusField.text = tp.radius.ToString();
            curvatureField.text = tp.curvature.ToString();
            twistField.text = tp.twistFromParent.ToString();
        }

        public void SetParamsFromFields()
        {
            if (selected)
            {
                selected.length = float.Parse(lengthField.text);
                selected.radius = float.Parse(radiusField.text);
                selected.curvature = float.Parse(curvatureField.text);
                selected.twistAngle = float.Parse(twistField.text);
            }
        }

        public void ApplyChangedParameters()
        {
            if (selected)
            {
                TelescopingSegment segment = selected.containingSegment;
                List<TelescopeParameters> allParams = segment.getParamList();
                TelescopeUtils.growChainToFit(allParams);
                segment.MakeAllShells(allParams);
                segment.SetShellExtensions(1);
                selected = null;
            }
        }

        public void ShrinkToFit()
        {
            if (selected)
            {
                TelescopingSegment segment = selected.containingSegment;
                List<TelescopeParameters> allParams = segment.getParamList();
                TelescopeUtils.growChainToFit(allParams, shrinkFit: true);
                segment.MakeAllShells(allParams);
                segment.SetShellExtensions(1);
                selected = null;
            }
        }

        public void ExtrudeNewShell()
        {
            if (selected)
            {
                if (selected.IsTerminal())
                {
                    TelescopingSegment segment = selected.containingSegment;

                    TelescopeParameters parentParams = selected.getParameters();
                    TelescopeParameters childParams = parentParams + segment.DefaultChildDiff;
                    if (childParams.radius < childParams.thickness || childParams.length < childParams.thickness) return;

                    TelescopingShell childShell = segment.addChildShell(selected, parentParams, childParams);
                    childShell.extendToRatio(1f, 2f);
                    SelectShell(childShell);
                }
            }
        }

        public void SetNumImpulses()
        {
            numImpulses = int.Parse(impulsePoints.text);
        }

        public void ShootSphere()
        {
            GameObject proj = Instantiate<GameObject>(projectilePrefab);
            Rigidbody rb = proj.GetComponent<Rigidbody>();

            Vector3 dir = Camera.main.transform.forward;

            Vector3 pos = Camera.main.transform.position;



            proj.transform.position = pos + 2 * dir;
            rb.velocity = 10 * dir;
            shootTime = 0;
        }
    }

    public enum DesignerMode
    {
        Shell, Curve, Curvature
    }
}