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

        public Material defaultTelescopeMaterial;
        public Material selectedTelescopeMaterial;

        public InputCurve curve;

        public GameObject projectilePrefab;

        private SplineControlPoint selectedControlPt;
        private float selectedDepth;
        
        private TreeControlPoint selectedTreePt;

        public LayerMask normalMask;
        public LayerMask curveModeMask;

        private float shootTime = 0f;
        private float shootDelay = 0.01f;

        private Vector3 lastMousePos;

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
        }

        void RaycastShells(Vector3 clickPos)
        {
            Ray mouseRay = Camera.main.ScreenPointToRay(clickPos);
            RaycastHit hitInfo = new RaycastHit();

            LayerMask mask = (currentMode == DesignerMode.Shell) ? normalMask : curveModeMask;

            if (Physics.Raycast(mouseRay, out hitInfo, 20f, mask))
            {
                TelescopingShell selection = hitInfo.collider.GetComponent<TelescopingShell>();
                SplineControlPoint controlPt = hitInfo.collider.GetComponent<SplineControlPoint>();
                TreeControlPoint treePt = hitInfo.collider.GetComponent<TreeControlPoint>();
                if (selection) SelectShell(selection);
                else if (treePt)
                {
                    treePt.containingTree.SelectNode(treePt);
                    selectedTreePt = treePt;
                    selectedDepth = Camera.main.WorldToScreenPoint(treePt.transform.position).z;
                }
                else if (controlPt)
                {
                    selectedControlPt = controlPt;
                    selectedDepth = Camera.main.WorldToScreenPoint(controlPt.transform.position).z;
                }
            }
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
            }

            else if (Input.GetButtonDown("Cancel"))
            {
                Deselect();
            }

            else if (Input.GetButtonDown("Submit"))
            {
                if (curve.points.Count >= 2)
                {
                    Vector3 point1 = curve.points[0].position;
                    float radius1 = curve.points[0].radius;
                    Vector3 point2 = curve.points[1].position;
                    float radius2 = curve.points[1].radius;

                    Vector3 startPt, endPt;
                    float startRadius, endRadius;

                    if (radius1 > radius2)
                    {
                        startRadius = radius1;
                        endRadius = radius2;
                        startPt = point1;
                        endPt = point2;
                    }
                    else
                    {
                        startRadius = radius2;
                        endRadius = radius1;
                        startPt = point2;
                        endPt = point1;
                    }

                    TelescopeUtils.telescopeOfCone(startPt, startRadius, endPt, endRadius);
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
                selected.curvatureAmount = float.Parse(curvatureField.text);
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
        Shell, Curve
    }
}