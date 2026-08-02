using System;
using System.Collections.Generic;
using UnityEngine;

public class FloorGenerator : MonoBehaviour
{
	public float LevelOneHeight = 1.0f; // Height of the first floor in Unity units
	public float LevelTwoHeight = 1.0f + 10f; // Height of the second floor in Unity units

	private float LevelOneWallHeight = 93.5f; // Height of the first floor walls in Unity units (93.5)

	// NEW: Organizational parents – created once in Start()
	private GameObject floorSectionContainer;
	private GameObject vertexCubeContainer;
	private GameObject wallSectionContainer;


	void Start()
	{
		new RoomIndex().Start();          // ← actually populate the list

		// NEW: Create the three organizational containers once, before any generation
		floorSectionContainer = CreateContainer("obj_FloorSectionContainer");
		vertexCubeContainer		= CreateContainer("obj_VertexCubeContainer");
		wallSectionContainer  = CreateContainer("obj_WallSectionContainer");

		foreach (RoomIndex.FloorSection floor in RoomIndex.All_FloorSectionList)
		{
			if (floor != null)
			{
				GenerateFloor(floor);
			}
		}

		GenerateVertexCube(RoomIndex.All_FloorSectionList);

		foreach (RoomIndex.WallSection wall in RoomIndex.All_WallSectionList)
		{
			if (wall != null)
			{
				GenerateWall(wall);
			}
		}
	}


	/// <summary>
	/// Creates a new empty GameObject at the origin to act as an organizational parent.
	/// </summary>
	private GameObject CreateContainer(string containerName)
	{
		GameObject container = new GameObject(containerName);
		container.transform.position = Vector3.zero;
		container.transform.rotation = Quaternion.identity;
		return container;
	}


	public void GenerateFloor(RoomIndex.FloorSection floor)
	{
		float width  = floor.Width;
		float length = floor.Length;

		GameObject floorTile = GameObject.CreatePrimitive(PrimitiveType.Cube);
		floorTile.name = floor.Name;

		// CHANGED: Parent under the pre-created container
		floorTile.transform.SetParent(floorSectionContainer.transform, worldPositionStays: false);

		// Move the center so the lowest-left corner lands on SpawnPos
		// (X = width, Z = length, Y = thickness)
		floorTile.transform.localPosition = floor.SpawnPos
											+ new Vector3(width * 0.5f, LevelOneHeight, length * 0.5f);
		floorTile.transform.localScale = new Vector3(width, 0.1f, length);

		// Turn off 2nd FloorSection if it exists, to avoid overlapping colors
		if (floor.Level == 2) { floorTile.SetActive(false); }

		foreach (var section in RoomIndex.All_FloorSectionList)
		{
			if (section != null && section.ID == floor.ID)
			{
				int colorIndex = section.ID % FloorSectionColors.Length;
				floorTile.GetComponent<Renderer>().material.color = FloorSectionColors[colorIndex];
				break;
			}
		}
	}


	public void GenerateVertexCube(List<RoomIndex.FloorSection> topList)
	{
		foreach (RoomIndex.FloorSection floor in topList)
		{
			foreach (var vertex in floor.VxList)
			{
				GameObject vertexCube = GameObject.CreatePrimitive(PrimitiveType.Cube);
				vertexCube.name = vertex.Name;

				// CHANGED: Parent under the pre-created container
				vertexCube.transform.SetParent(vertexCubeContainer.transform, worldPositionStays: false);

				vertexCube.transform.localPosition = new Vector3(vertex.Position.x, vertex.Position.y, vertex.Position.z);

				// Turn off 2nd FloorSection if it exists, to avoid overlapping colors
				if (vertex.Level == 2)
				{
					vertexCube.transform.localPosition = new Vector3(vertex.Position.x, vertex.Position.y + LevelTwoHeight, vertex.Position.z);
					vertexCube.SetActive(false);
					vertex.IsActive = false;
				}

				vertexCube.transform.localScale = new Vector3(3f, 3f, 3f);
				vertexCube.GetComponent<Renderer>().material.color = Color.red;
			}
		}
	}


	public void GenerateWall(RoomIndex.WallSection wall)
	{
		float width  = wall.Width;
		float length = wall.Length;

		GameObject wallTile = GameObject.CreatePrimitive(PrimitiveType.Cube);
		wallTile.name = wall.Name;

		// CHANGED: Parent under the pre-created container
		wallTile.transform.SetParent(wallSectionContainer.transform, worldPositionStays: false);

		// Move the center so the lowest-left corner lands on SpawnPos
		// (X = width, Z = length, Y = thickness)
		wallTile.transform.localPosition = wall.SpawnPos
											+ new Vector3(width * 0.5f, LevelOneWallHeight * 0.5f, length * 0.5f);
		wallTile.transform.localScale = new Vector3(width, LevelOneWallHeight, length);

		// Turn off 2nd FloorSection if it exists, to avoid overlapping colors
		if (wall.Level == 2) { wallTile.SetActive(false); }

		Material wallMat = wallTile.GetComponent<Renderer>().material;

		SetMaterialTransparency(wallMat, 0.5f); // Set alpha to 0.5 for transparency

		Color currentColor = Color.whiteSmoke; // Default color
		currentColor.a = 0.5f; // Set alpha to 0.5 for transparency
		wallMat.color = currentColor;

		wallTile.GetComponent<Renderer>().material = wallMat;
	}





	public void SetMaterialTransparency(Material material, float alpha)
	{
		// --- Force Transparent mode (Built-in Render Pipeline) ---
		material.SetFloat("_Mode", 3); // 3 = Transparent
		material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
		material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
		material.SetInt("_ZWrite", 0);
		material.DisableKeyword("_ALPHATEST_ON");
		material.EnableKeyword("_ALPHABLEND_ON");
		material.DisableKeyword("_ALPHAPREMULTIPLY_ON");
		material.renderQueue = 3000;
		Color currentColor = material.color;
		currentColor.a = alpha; // Set alpha to the specified value
		material.color = currentColor;
	}





	public static readonly Color32[] FloorSectionColors = new Color32[]
	{
		new Color32(255,  50,  50, 255), //  0 Bright Red
		new Color32( 50, 180, 255, 255), //  1 Sky Blue
		new Color32( 50, 220,  50, 255), //  2 Lime Green
		new Color32(255, 180,   0, 255), //  3 Orange
		new Color32(180,  50, 255, 255), //  4 Violet
		new Color32(  0, 220, 180, 255), //  5 Turquoise
		new Color32(255, 100, 180, 255), //  6 Hot Pink
		new Color32(100, 100, 255, 255), //  7 Indigo
		new Color32(220, 220,  40, 255), //  8 Yellow
		new Color32( 40, 160, 120, 255), //  9 Teal
		new Color32(255, 140,  80, 255), // 10 Coral
		new Color32(140,  40, 180, 255), // 11 Purple
		new Color32( 80, 220,  80, 255), // 12 Bright Green
		new Color32(255,  80, 120, 255), // 13 Rose
		new Color32( 60, 120, 255, 255), // 14 Royal Blue
		new Color32(200, 160,  40, 255), // 15 Gold
		new Color32(180,  80,  40, 255), // 16 Brown-Orange
		new Color32( 40, 200, 220, 255), // 17 Cyan
		new Color32(220,  40, 160, 255), // 18 Magenta
		new Color32(120, 220, 160, 255), // 19 Mint
		new Color32(255, 160, 200, 255), // 20 Light Pink
		new Color32(100,  60, 200, 255), // 21 Deep Violet
		new Color32(160, 220,  40, 255), // 22 Chartreuse
		new Color32(220, 100,  40, 255), // 23 Burnt Orange
		new Color32( 40, 100, 180, 255), // 24 Steel Blue
		new Color32(200,  40,  80, 255), // 25 Crimson
		new Color32( 80, 180,  40, 255), // 26 Forest Green
		new Color32(180, 140, 255, 255), // 27 Lavender
		new Color32(255, 220, 100, 255), // 28 Light Gold
		new Color32( 40, 160, 200, 255), // 29 Ocean
		new Color32(220,  80, 220, 255), // 30 Orchid
		new Color32(140, 100,  60, 255), // 31 Tan / Earth
	};
}