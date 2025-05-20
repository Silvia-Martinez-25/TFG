using UnityEngine;

public class Articulations : MonoBehaviour
{
    public float stiffness = 10000f;
    public float forceLimit = 1000f;
    public ArticulationDriveType driveType = ArticulationDriveType.Target;

    public GameObject root;

    private bool initialized = false;

    // Start is called before the first frame update
    private void setValues()
    {
        foreach (ArticulationBody child in root.GetComponentsInChildren<ArticulationBody>())
        {
            ArticulationDrive drive = child.xDrive;

            //Confriguraciones los parametros
            drive.stiffness = stiffness;
            drive.forceLimit = forceLimit;
            drive.target = 0.0f;
            drive.targetVelocity = 0.0f;
            drive.driveType = driveType;

            //Asigna el drive modificado
            child.xDrive = drive;
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (!initialized)
        {
            setValues();
            initialized = true;
        }
    }
}
