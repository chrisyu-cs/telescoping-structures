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

        public Material defaultLineMaterial;

        public InputCurve curve;

        public GameObject projectilePrefab;
        
        private float selectedDepth;

        private DraggablePoint draggable;

        public LayerMask normalMask;
        public LayerMask curveModeMask;
        public LayerMask curvatureModeMask;

        public InputField filenameField;

        private float shootTime = 0f;
        private float shootDelay = 0.05f;

        private Vector3 lastMousePos;

        public Toggle sparseToggle;
        public bool UseSparseSolve = false;

        public int numImpulses = 10;
        public SplineCanvas splineCanvas;

        public MeshFilter currentMesh;

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
            splineCanvas = FindObjectOfType<SplineCanvas>();
        }

        bool RaycastShells(Vector3 clickPos, out RaycastHit hitInfo)
        {
            Ray mouseRay = Camera.main.ScreenPointToRay(clickPos);
            hitInfo = new RaycastHit();

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
                return true;
            }

            return false;
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

            RaycastHit hitInfo = new RaycastHit();

            if (Input.GetMouseButtonDown(0) && RaycastShells(Input.mousePosition, out hitInfo))
            {
                TelescopingShell selection = hitInfo.collider.GetComponent<TelescopingShell>();
                DraggablePoint draggablePt = hitInfo.collider.GetComponent<DraggablePoint>();

                if (selection) SelectShell(selection);

                else if (draggablePt)
                {
                    draggable = draggablePt;
                    selectedDepth = Camera.main.WorldToScreenPoint(draggablePt.transform.position).z;
                }
            }

            else if (Input.GetMouseButton(0) && draggable)
            {
                Vector3 clickPos = Input.mousePosition;
                clickPos.z = selectedDepth;
                Vector3 worldPos = Camera.main.ScreenToWorldPoint(clickPos);

                DraggablePoint intersectedBulb = splineCanvas.IntersectedBulb(worldPos);

                if (draggable.Type != PointType.Bulb && draggable.IsEndPoint() && intersectedBulb)
                {
                    draggable.AttachToBulb(intersectedBulb);
                }
                else
                {
                    draggable.Move(worldPos);
                }
            }

            else if (Input.GetMouseButtonDown(1) && RaycastShells(Input.mousePosition, out hitInfo))
            {
                Vector3 clickPos = Input.mousePosition;
                DraggablePoint clicked = hitInfo.collider.GetComponent<DraggablePoint>();

                if (clicked)
                {
                    if (Input.GetKey("left ctrl")) clicked.Delete();
                    else clicked.Duplicate();
                }

                // Store the eye-space position of the click.
                // Use eye space because we always want moving up/down to correspond
                // to bigger or smaller, regardless of camera orientation.
                lastMousePos = clickPos;
                lastMousePos.z = selectedDepth;
                lastMousePos = Camera.main.ScreenToViewportPoint(lastMousePos);
            }

            else if (Input.GetMouseButtonDown(2) && RaycastShells(Input.mousePosition, out hitInfo))
            {
                Vector3 clickPos = Input.mousePosition;
                
                // Store the eye-space position of the click.
                // Use eye space because we always want moving up/down to correspond
                // to bigger or smaller, regardless of camera orientation.
                lastMousePos = clickPos;
                lastMousePos.z = selectedDepth;
                lastMousePos = Camera.main.ScreenToViewportPoint(lastMousePos);
            }

            else if (Input.mouseScrollDelta.y != 0 && RaycastShells(Input.mousePosition, out hitInfo))
            {
                DraggablePoint draggablePt = hitInfo.collider.GetComponent<DraggablePoint>();
                if (draggablePt)
                {
                    float change = Input.mouseScrollDelta.y * 0.05f;
                    draggablePt.Resize(change);
                }
            }

            else if (Input.GetKeyDown("delete") && RaycastShells(Input.mousePosition, out hitInfo))
            {
                DraggablePoint draggablePt = hitInfo.collider.GetComponent<DraggablePoint>();

                if (draggablePt)
                {
                    draggablePt.Delete();
                }
            }

            else if (Input.GetMouseButtonUp(0) || Input.GetMouseButtonUp(2))
            {
                draggable = null;
            }

            else if (Input.GetButtonDown("Cancel"))
            {
                Deselect();
            }

            else if (Input.GetButtonDown("Submit"))
            {
                Debug.Log("submit");
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

        public void WriteCurrentMeshOBJ()
        {
            if (filenameField.text == "")
            {
                Debug.Log("file name empty");
                return;
            }
            OBJWriter.ExportToOBJ(currentMesh.mesh, filenameField.text);
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