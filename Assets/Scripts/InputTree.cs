using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public class InputTree : MonoBehaviour
    {
        public Material lineMaterial;
        public Material defaultMaterial;
        public Material highlightedMaterial;
        private TreeControlPoint rootNode;

        private TreeControlPoint highlightedNode;

        // Use this for initialization
        void Start()
        {
            GameObject rootObj = new GameObject();
            rootObj.transform.parent = transform;
            rootObj.transform.localPosition = Vector3.zero;
            rootObj.name = "root";

            rootNode = rootObj.AddComponent<TreeControlPoint>();
            rootNode.containingTree = this;
        }

        public void SelectNode(TreeControlPoint pt)
        {
            Deselect();
            pt.SetMaterial(highlightedMaterial);
            highlightedNode = pt;
        }

        public void Deselect()
        {
            if (highlightedNode)
            {
                highlightedNode.SetMaterial(defaultMaterial);
                highlightedNode = null;
            }
        }

        public void MakeTelescopes()
        {

        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown("h"))
            {
                rootNode.AddNewChild();
            }
        }
    }

}