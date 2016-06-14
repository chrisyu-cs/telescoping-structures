using UnityEngine;
using System.Collections.Generic;

using System;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
using System.IO;

namespace Telescopes
{
    public class InputTree : MonoBehaviour
    {
        public Material lineMaterial;
        public Material defaultMaterial;
        public Material highlightedMaterial;
        private TreeControlPoint rootNode;

        private TreeControlPoint highlightedNode;

        public TreeControlPoint Root
        {
            get { return rootNode; }
        }

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
            List<TelescopeElement> elements = new List<TelescopeElement>();
            if (rootNode)
            {
                rootNode.MakeTelescopesToChildren(elements, null);
            }
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown("h"))
            {
                highlightedNode.AddNewChild();
            }

            if (Input.GetKeyDown("p"))
            {
                NodeSerialized ns = new NodeSerialized(this);
                ns.Print();
            }
        }

        public void Save()
        {
            string filename = DesignerController.instance.filenameField.text;
            if (filename.Length < 1) return;
            Debug.Log("Save to " + filename);

            NodeSerialized treeRoot = new NodeSerialized(this);

            IFormatter formatter = new BinaryFormatter();
            Stream stream = new FileStream(filename, FileMode.Create, FileAccess.Write, FileShare.None);
            formatter.Serialize(stream, treeRoot);
            stream.Close();
        }

        public void Load()
        {
            string filename = DesignerController.instance.filenameField.text;
            if (filename.Length < 1) return;
            Debug.Log("Load from " + filename);

            IFormatter formatter = new BinaryFormatter();
            Stream stream = new FileStream(filename, FileMode.Open, FileAccess.Read, FileShare.Read);
            NodeSerialized treeRoot = (NodeSerialized)formatter.Deserialize(stream);
            stream.Close();

            ReconstructTree(treeRoot);
        }

        void ReconstructTree(NodeSerialized rootSerialized)
        {
            for (int i = 0; i < transform.childCount; i++)
            {
                Transform child = transform.GetChild(i);
                Destroy(child.gameObject);
            }
            
            GameObject rootObj = new GameObject();
            rootObj.transform.parent = transform;
            rootObj.transform.localPosition = Vector3.zero;
            rootObj.name = "root";

            rootNode = rootObj.AddComponent<TreeControlPoint>();
            rootNode.containingTree = this;
            rootNode.RebuildFromSerialized(rootSerialized);
        }
    }


    [Serializable]
    public class NodeSerialized
    {
        public float positionX, positionY, positionZ;

        public float radius;
        public float curvaturePointX, curvaturePointY, curvaturePointZ;
        public List<NodeSerialized> children;

        public NodeSerialized(InputTree tree)
            : this(tree.Root)
        {

        }

        public NodeSerialized(TreeControlPoint rootNode)
        {
            positionX = rootNode.node.position.x;
            positionY = rootNode.node.position.y;
            positionZ = rootNode.node.position.z;
            radius = rootNode.node.radius;

            curvaturePointX = rootNode.curvaturePoint().x;
            curvaturePointY = rootNode.curvaturePoint().y;
            curvaturePointZ = rootNode.curvaturePoint().z;

            children = new List<NodeSerialized>();
            foreach (var child in rootNode.children)
            {
                children.Add(new NodeSerialized(child));
            }
        }

        public void Print()
        {

            Vector3 position = new Vector3(positionX, positionY, positionZ);
            Vector3 curvaturePoint = new Vector3(curvaturePointX, curvaturePointY, curvaturePointZ);

            Debug.Log("position = " + position + ", radius = " + radius + ", curvature pt = " + curvaturePoint);
            foreach (var child in children)
            {
                child.Print();
            }
        }
    }
}