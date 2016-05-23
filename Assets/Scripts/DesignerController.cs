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

        public Material defaultTelescopeMaterial;
        public Material selectedTelescopeMaterial;

        void Awake()
        {
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
            currentMode = DesignerMode.View;
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetButtonDown("ToggleMode"))
            {
                if (currentMode == DesignerMode.View)
                {
                    currentMode = DesignerMode.Edit;
                    modeText.text = "Edit mode";
                }
                else if (currentMode == DesignerMode.Edit)
                {
                    currentMode = DesignerMode.View;
                    modeText.text = "View mode";
                }
            }

            if (Input.GetMouseButtonDown(0))
            {
                Vector3 clickPos = Input.mousePosition;
                Ray mouseRay = Camera.main.ScreenPointToRay(clickPos);
                RaycastHit hitInfo = new RaycastHit();
                if (Physics.Raycast(mouseRay, out hitInfo, 20f))
                {
                    TelescopingShell selection = hitInfo.collider.GetComponent<TelescopingShell>();
                    SelectShell(selection);
                }
            }

            else if (Input.GetButtonDown("Cancel"))
            {
                Deselect();
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
    }

    public enum DesignerMode
    {
        View, Edit
    }
}