using System;
using System.Collections.Generic;
using UnityEngine;

namespace CondoUnityModel
{

    //? ChatGPT Conversion Link: https://chatgpt.com/share/695b374f-f3f4-8008-87e3-786437ffd0cb 
    // 
    // Note: The following code defines a data model for representing building structures in Unity, focusing on rooms, walls, junctions, and ceiling profiles. It includes utilities for unit conversion and a builder for constructing the building graph from placed rectangular rooms.

    /// <summary>
    /// Integer "sixteenths of an inch" everywhere for precision and consistency.
    /// 1 inch = 16 units.
    /// </summary>
    public static class Units16
    {
        public const int PerInch = 16;
        public const int PerFoot = 12 * PerInch;

        public static int Inches(int inches) => inches * PerInch;
        public static int Feet(int feet) => feet * PerFoot;
        public static int FeetInches(int feet, int inches) => (feet * 12 + inches) * PerInch;

        public static float ToInchesFloat(int u16) => u16 / (float)PerInch;
        public static float ToFeetFloat(int u16) => u16 / (float)PerFoot;
    }

    [Serializable]
    public struct EntityId : IEquatable<EntityId>
    {
        [SerializeField] private string value;
        public string Value => value;

        public EntityId(string v) { value = v; }
        public static EntityId NewId() => new EntityId(Guid.NewGuid().ToString("N"));

        public bool Equals(EntityId other) => value == other.value;
        public override bool Equals(object obj) => obj is EntityId other && Equals(other);
        public override int GetHashCode() => value != null ? value.GetHashCode() : 0;
        public override string ToString() => value;
    }

    // -------------------------
    // Rooms
    // -------------------------

    public enum RoomSide { North, South, East, West }
    public enum WallFace { FaceA, FaceB }

    [Serializable]
    public class Room
    {
        public EntityId Id = EntityId.NewId();
        public string Name;

        /// <summary>
        /// Interior clear size (drywall-to-drywall) in 1/16" units.
        /// Example 10'x10' => (120in*16, 120in*16) => (1920, 1920)
        /// </summary>
        public Vector2Int InteriorSizeU16;

        /// <summary>
        /// Optional floor elevation (finished floor) in 1/16" units, relative to building datum.
        /// </summary>
        public int FloorElevationU16 = 0;

        /// <summary>
        /// Boundary walls referenced by this room (rooms do not own walls).
        /// </summary>
        public List<RoomWallRef> BoundaryWalls = new();

        /// <summary>
        /// Room footprint corners / vertices in order (clockwise or CCW), expressed as Junction IDs.
        /// This makes ceiling updates easy: change a junction order and everything remains referenced by ID.
        /// </summary>
        public List<EntityId> FootprintJunctionIds = new();

        /// <summary>
        /// Vaulted ceiling model: store ceiling height at the footprint vertices (and optional extra vertices like ridge points),
        /// plus explicit triangles for accurate rendering. This is the most practical: edit heights directly, and display is exact.
        /// </summary>
        public CeilingMeshProfile Ceiling = new CeilingMeshProfile();
    }

    [Serializable]
    public class RoomWallRef
    {
        public RoomSide Side;
        public EntityId WallId;

        /// <summary>
        /// Which face of the wall is interior to THIS room (important for boxes/outlets).
        /// </summary>
        public WallFace InteriorFace;
    }

    // -------------------------
    // Junctions (plan topology)
    // -------------------------

    public enum JunctionType { CornerL, Tee, Cross, End, Custom }

    [Serializable]
    public class Junction
    {
        public EntityId Id = EntityId.NewId();
        public JunctionType Type = JunctionType.CornerL;

        /// <summary>
        /// Position in plan space in 1/16" units.
        /// Treat this as the authoritative topology position for corners/intersections.
        /// </summary>
        public Vector2Int PlanPositionU16;

        /// <summary>
        /// Walls connected to this junction.
        /// </summary>
        public List<EntityId> ConnectedWallIds = new();
    }

    // -------------------------
    // Ceiling (vaulted) representation
    // -------------------------

    public enum CeilingVertexKind { FootprintCorner, Ridge, Custom }

    [Serializable]
    public class CeilingMeshProfile
    {
        /// <summary>
        /// Ceiling vertices are stored in room-local 2D (X,Y) (in plan) and a height above the ROOM FLOOR.
        /// X/Y are in 1/16" and should match your room-local plan coordinates.
        /// HeightU16 is also 1/16".
        ///
        /// Practical workflow:
        /// - For rectangles: use 4 footprint corners (kind=FootprintCorner)
        /// - For a gable: add 2 ridge vertices (kind=Ridge) and adjust triangles
        /// - For complex shapes: add more vertices and triangles
        /// </summary>
        public List<CeilingVertex> Vertices = new();

        /// <summary>
        /// Explicit triangles (indices into Vertices). This guarantees accurate display and avoids complex polygon math.
        /// Default for a quad: [0,1,2, 0,2,3]
        /// </summary>
        public List<int> Triangles = new();

        /// <summary>
        /// If true and Triangles is empty, we will auto-fill a simple quad triangulation (0,1,2)(0,2,3),
        /// assuming Vertices[0..3] are the four corners in order.
        /// </summary>
        public bool AutoTriangulateQuadIfEmpty = true;
    }

    [Serializable]
    public class CeilingVertex
    {
        public CeilingVertexKind Kind = CeilingVertexKind.Custom;

        /// <summary>
        /// Room-local plan position in 1/16" units (X=along room local right, Y=along room local forward).
        /// </summary>
        public Vector2Int LocalPlanU16;

        /// <summary>
        /// Height above the room finished floor in 1/16" units.
        /// </summary>
        public int HeightAboveFloorU16;
    }

    // -------------------------
    // Walls
    // -------------------------

    public enum WallKind { Interior, Exterior }

    [Serializable]
    public class Wall
    {
        public EntityId Id = EntityId.NewId();
        public string Name;

        public WallKind Kind = WallKind.Interior;

        /// <summary>
        /// Wall is defined by two junctions in plan.
        /// </summary>
        public EntityId StartJunctionId;
        public EntityId EndJunctionId;

        /// <summary>
        /// Room references: RoomA is on FaceA side, RoomB on FaceB side.
        /// For exterior walls, one side can be empty.
        /// </summary>
        public EntityId RoomAId;
        public EntityId RoomBId;

        /// <summary>
        /// Construction dimensions in 1/16".
        /// Typical interior partition might be ~5".
        /// </summary>
        public int ThicknessU16 = Units16.Inches(5);

        /// <summary>
        /// Default wall height if ceiling is flat or if you want a fallback.
        /// For vaulted ceilings, the *effective top profile* should be derived from the adjacent room ceiling mesh.
        /// </summary>
        public int DefaultHeightU16 = Units16.FeetInches(8, 0);

        public StudRule Studs = new StudRule();
        public List<WallFeature> Features = new();
    }

    [Serializable]
    public class StudRule
    {
        public bool Enabled = true;
        public int SpacingOnCenterU16 = Units16.Inches(16);
        public int StudWidthU16 = Units16.Inches(2); // keep simple; if you want exact 1.5" => Units16.Inches(1) + Units16.Inches(1)/2
        public bool IncludeDoubleTopPlate = true;
        public bool IncludeBottomPlate = true;
    }

    // -------------------------
    // Wall Features
    // -------------------------

    public enum FeatureCategory { Electrical, Plumbing, Framing, HVAC, Other }

    [Serializable]
    public abstract class WallFeature
    {
        public EntityId Id = EntityId.NewId();
        public string Label;
        public FeatureCategory Category;

        /// <summary>
        /// Local placement in wall coordinate system:
        /// X = along wall from StartJunction toward EndJunction
        /// Y = height from finished floor
        /// Z = into wall thickness (0..Thickness)
        /// All in 1/16".
        /// </summary>
        public Vector3Int LocalU16;

        /// <summary>
        /// Axis-aligned size in 1/16".
        /// </summary>
        public Vector3Int SizeU16;

        /// <summary>
        /// Which face this feature is associated with.
        /// </summary>
        public WallFace Face;
    }

    public enum ElectricalBoxType { Outlet, Switch, JunctionBox, Data, Coax, Other }

    [Serializable]
    public class ElectricalBoxFeature : WallFeature
    {
        public ElectricalBoxType BoxType;
        public bool Gfci;
        public bool Usb;
        public int Amps; // 15/20 etc.
        public int GangCount = 1;
    }

    public enum PipeType { WaterSupply, Drain, Vent, Gas, Other }

    [Serializable]
    public class PipeFeature : WallFeature
    {
        public PipeType Type;
        public int OuterDiameterU16;
        public bool IsHot;
        public bool IsVent;
    }

    public enum StudOverrideKind { AddStud, RemoveStud, DoubleStud, Header, KingStud, JackStud }

    [Serializable]
    public class StudOverrideFeature : WallFeature
    {
        public StudOverrideKind Kind;
    }

    // -------------------------
    // Project Container (ScriptableObject)
    // -------------------------

    [CreateAssetMenu(menuName = "Condo Unity Model/Building Graph", fileName = "BuildingGraph")]
    public class BuildingGraph : ScriptableObject
    {
        public List<Room> Rooms = new();
        public List<Wall> Walls = new();
        public List<Junction> Junctions = new();

        public Room FindRoom(EntityId id) => Rooms.Find(r => r.Id.Equals(id));
        public Wall FindWall(EntityId id) => Walls.Find(w => w.Id.Equals(id));
        public Junction FindJunction(EntityId id) => Junctions.Find(j => j.Id.Equals(id));

        // -------------------------
        // Practical ceiling -> mesh helper
        // -------------------------

        /// <summary>
        /// Creates a Unity Mesh for the room ceiling using the room's CeilingMeshProfile.
        /// Vertex positions are returned in ROOM-LOCAL space:
        /// (x, y, z) where y is height, and x/z are plan axes (Unity convention).
        /// All converted from U16 to Unity units (1 Unity unit = 1 inch by your earlier choice).
        /// </summary>
        public Mesh BuildRoomCeilingMesh(Room room)
        {
            if (room == null) throw new ArgumentNullException(nameof(room));
            var prof = room.Ceiling;
            if (prof == null) throw new InvalidOperationException("Room.Ceiling is null.");
            if (prof.Vertices == null || prof.Vertices.Count < 3)
                throw new InvalidOperationException("Ceiling profile must have at least 3 vertices.");

            var tris = prof.Triangles ?? new List<int>();
            if (tris.Count == 0 && prof.AutoTriangulateQuadIfEmpty && prof.Vertices.Count >= 4)
            {
                // Simple and very common: rectangular rooms.
                tris = new List<int> { 0, 1, 2, 0, 2, 3 };
            }
            if (tris.Count < 3) throw new InvalidOperationException("Ceiling triangles are empty/invalid.");

            // Convert U16 -> Unity inches (your grid: 1 unit = 1 inch).
            // So 1 Unity unit == 1 inch => divide U16 by 16.
            float U16ToUnity = 1f / Units16.PerInch;

            var verts = new Vector3[prof.Vertices.Count];
            for (int i = 0; i < prof.Vertices.Count; i++)
            {
                var v = prof.Vertices[i];
                float x = v.LocalPlanU16.x * U16ToUnity;                 // plan X
                float y = v.HeightAboveFloorU16 * U16ToUnity;            // height
                float z = v.LocalPlanU16.y * U16ToUnity;                 // plan Y -> Unity Z
                verts[i] = new Vector3(x, y, z);
            }

            var mesh = new Mesh();
            mesh.name = $"{room.Name}_Ceiling";
            mesh.vertices = verts;
            mesh.triangles = tris.ToArray();
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }
    }


    /// <summary>
    /// Part B: Adjacency resolution + auto-creation/merging of Junctions/Walls from axis-aligned room rectangles.
    ///
    /// Assumptions (kept practical for Version 1):
    /// - Rooms are rectangles in plan (no rotated rooms).
    /// - Rooms are placed on a shared plan grid (U16 units).
    /// - Room positions are given as interior clear rectangles (drywall-to-drywall).
    /// - Wall thickness is handled as construction data on the Wall, and shared walls are single Wall objects.
    ///
    /// Outputs:
    /// - Junctions created/merged by exact plan position.
    /// - Walls created/merged by (startJunction,endJunction) ignoring direction.
    /// - Each Room gets 4 boundary wall refs (N/S/E/W) with InteriorFace correct.
    /// - Each Wall gets RoomA/RoomB assigned consistently (FaceA/FaceB).
    /// </summary>
    public static class BuildingGraphBuilder
    {
        // -------------------------
        // Public API
        // -------------------------

        public static void BuildFromPlacedRectRooms(
            BuildingGraph graph,
            IReadOnlyList<PlacedRectRoom> placedRooms,
            int defaultWallThicknessU16)
        {
            if (graph == null) throw new ArgumentNullException(nameof(graph));
            if (placedRooms == null) throw new ArgumentNullException(nameof(placedRooms));

            graph.Rooms.Clear();
            graph.Walls.Clear();
            graph.Junctions.Clear();

            // Indexes for merging
            var junctionByPos = new Dictionary<Vector2Int, EntityId>();
            var wallByKey = new Dictionary<UndirectedWallKey, EntityId>();

            // Create rooms first (IDs stable for later linking)
            var roomPlacements = new List<RoomPlacementRuntime>(placedRooms.Count);
            foreach (var pr in placedRooms)
            {
                var room = new Room
                {
                    Id = EntityId.NewId(),
                    Name = pr.Name,
                    InteriorSizeU16 = pr.SizeU16,
                    FloorElevationU16 = pr.FloorElevationU16,
                    BoundaryWalls = new List<RoomWallRef>(),
                    FootprintJunctionIds = new List<EntityId>(),
                    Ceiling = pr.CeilingProfile ?? new CeilingMeshProfile()
                };
                graph.Rooms.Add(room);

                roomPlacements.Add(new RoomPlacementRuntime
                {
                    Room = room,
                    Min = pr.MinU16,
                    Max = pr.MinU16 + pr.SizeU16
                });
            }

            // Pass 1: Create/merge junctions + create/merge perimeter walls for each room
            foreach (var rp in roomPlacements)
            {
                // Rectangle corners in plan (room-local not needed here)
                var bl = new Vector2Int(rp.Min.x, rp.Min.y);
                var br = new Vector2Int(rp.Max.x, rp.Min.y);
                var tr = new Vector2Int(rp.Max.x, rp.Max.y);
                var tl = new Vector2Int(rp.Min.x, rp.Max.y);

                // Junctions
                var jBL = GetOrCreateJunction(graph, junctionByPos, bl);
                var jBR = GetOrCreateJunction(graph, junctionByPos, br);
                var jTR = GetOrCreateJunction(graph, junctionByPos, tr);
                var jTL = GetOrCreateJunction(graph, junctionByPos, tl);

                // Store footprint (ordered)
                rp.Room.FootprintJunctionIds.Clear();
                rp.Room.FootprintJunctionIds.Add(jBL);
                rp.Room.FootprintJunctionIds.Add(jBR);
                rp.Room.FootprintJunctionIds.Add(jTR);
                rp.Room.FootprintJunctionIds.Add(jTL);

                // Perimeter walls (undirected merge)
                // South: BL -> BR
                var wS = GetOrCreateWall(graph, wallByKey, jBL, jBR, defaultWallThicknessU16);
                // East:  BR -> TR
                var wE = GetOrCreateWall(graph, wallByKey, jBR, jTR, defaultWallThicknessU16);
                // North: TR -> TL
                var wN = GetOrCreateWall(graph, wallByKey, jTR, jTL, defaultWallThicknessU16);
                // West:  TL -> BL
                var wW = GetOrCreateWall(graph, wallByKey, jTL, jBL, defaultWallThicknessU16);

                // Attach to room (InteriorFace computed later once Wall RoomA/RoomB is set)
                rp.Room.BoundaryWalls.Clear();
                rp.Room.BoundaryWalls.Add(new RoomWallRef { Side = RoomSide.South, WallId = wS, InteriorFace = WallFace.FaceA });
                rp.Room.BoundaryWalls.Add(new RoomWallRef { Side = RoomSide.East,  WallId = wE, InteriorFace = WallFace.FaceA });
                rp.Room.BoundaryWalls.Add(new RoomWallRef { Side = RoomSide.North, WallId = wN, InteriorFace = WallFace.FaceA });
                rp.Room.BoundaryWalls.Add(new RoomWallRef { Side = RoomSide.West,  WallId = wW, InteriorFace = WallFace.FaceA });
            }

            // Pass 2: Assign RoomA/RoomB on each wall by probing which rooms touch it
            // This also determines which face is "interior" for each room's boundary ref.
            ResolveWallRoomSides(graph, roomPlacements);

            // Pass 3: Update Junction.ConnectedWallIds lists
            RebuildJunctionConnectivity(graph);
        }

        // -------------------------
        // Placement input types
        // -------------------------

        [Serializable]
        public class PlacedRectRoom
        {
            public string Name;

            /// <summary>
            /// Interior clear rectangle origin in plan (U16).
            /// This is the "drywall-to-drywall" minimum corner in world plan coordinates.
            /// </summary>
            public Vector2Int MinU16;

            /// <summary>
            /// Interior clear size (U16).
            /// </summary>
            public Vector2Int SizeU16;

            public int FloorElevationU16 = 0;

            /// <summary>
            /// Optional vaulted ceiling profile already authored for the room.
            /// If null, an empty profile will be created and you can author later.
            /// </summary>
            public CeilingMeshProfile CeilingProfile;
        }

        private struct RoomPlacementRuntime
        {
            public Room Room;
            public Vector2Int Min;
            public Vector2Int Max;
        }

        // -------------------------
        // Junction and Wall creation/merge
        // -------------------------

        private static EntityId GetOrCreateJunction(
            BuildingGraph graph,
            Dictionary<Vector2Int, EntityId> junctionByPos,
            Vector2Int posU16)
        {
            if (junctionByPos.TryGetValue(posU16, out var id))
                return id;

            var j = new Junction
            {
                Id = EntityId.NewId(),
                PlanPositionU16 = posU16,
                Type = JunctionType.CornerL,
                ConnectedWallIds = new List<EntityId>()
            };
            graph.Junctions.Add(j);
            junctionByPos[posU16] = j.Id;
            return j.Id;
        }

        private static EntityId GetOrCreateWall(
            BuildingGraph graph,
            Dictionary<UndirectedWallKey, EntityId> wallByKey,
            EntityId aJunctionId,
            EntityId bJunctionId,
            int thicknessU16)
        {
            var key = new UndirectedWallKey(aJunctionId, bJunctionId);
            if (wallByKey.TryGetValue(key, out var wallId))
                return wallId;

            var w = new Wall
            {
                Id = EntityId.NewId(),
                Name = "Wall",
                Kind = WallKind.Interior,
                StartJunctionId = aJunctionId,
                EndJunctionId = bJunctionId,
                ThicknessU16 = thicknessU16,
                DefaultHeightU16 = Units16.FeetInches(8, 0),
                Features = new List<WallFeature>()
            };

            graph.Walls.Add(w);
            wallByKey[key] = w.Id;
            return w.Id;
        }

        private readonly struct UndirectedWallKey : IEquatable<UndirectedWallKey>
        {
            private readonly string _a;
            private readonly string _b;

            public UndirectedWallKey(EntityId j1, EntityId j2)
            {
                // Normalize direction: smaller string first
                var s1 = j1.Value ?? "";
                var s2 = j2.Value ?? "";
                if (string.CompareOrdinal(s1, s2) <= 0) { _a = s1; _b = s2; }
                else { _a = s2; _b = s1; }
            }

            public bool Equals(UndirectedWallKey other) => _a == other._a && _b == other._b;
            public override bool Equals(object obj) => obj is UndirectedWallKey other && Equals(other);
            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = 17;
                    hash = hash * 31 + _a.GetHashCode();
                    hash = hash * 31 + _b.GetHashCode();
                    return hash;
                }
            }
        }

        // -------------------------
        // Wall side resolution
        // -------------------------

        private static void ResolveWallRoomSides(BuildingGraph graph, List<RoomPlacementRuntime> placements)
        {
            // Build wall segment geometry (plan) for quick checks
            var junctionPos = new Dictionary<string, Vector2Int>();
            foreach (var j in graph.Junctions)
                junctionPos[j.Id.Value] = j.PlanPositionU16;

            foreach (var wall in graph.Walls)
            {
                var a = junctionPos[wall.StartJunctionId.Value];
                var b = junctionPos[wall.EndJunctionId.Value];

                bool vertical = a.x == b.x;
                bool horizontal = a.y == b.y;
                if (!vertical && !horizontal)
                    throw new InvalidOperationException("Version 1 builder supports only axis-aligned walls.");

                // Determine which rooms touch this wall (by rectangle edge coincidence).
                // For an interior shared wall, it will match exactly one edge of each of two rooms.
                // For exterior wall, only one room matches.
                RoomPlacementRuntime? hit1 = null;
                RoomPlacementRuntime? hit2 = null;

                foreach (var rp in placements)
                {
                    if (RoomTouchesWall(rp, a, b, vertical, horizontal))
                    {
                        if (hit1 == null) hit1 = rp;
                        else if (hit2 == null) { hit2 = rp; break; }
                    }
                }

                // Assign wall kind and sides
                if (hit1 == null)
                {
                    // No room touches it; should not happen in normal build
                    wall.Kind = WallKind.Interior;
                    continue;
                }

                if (hit2 == null)
                {
                    // Exterior (single-sided)
                    wall.Kind = WallKind.Exterior;
                    wall.RoomAId = hit1.Value.Room.Id;
                    wall.RoomBId = default; // empty
                }
                else
                {
                    wall.Kind = WallKind.Interior;

                    // Decide RoomA vs RoomB deterministically by which room lies on the "left/down" side.
                    // We define FaceA as the side you get when walking from Start->End and looking left (2D cross).
                    // For axis-aligned:
                    // - Horizontal wall (y constant): if direction is left->right, "left" is +Y (north).
                    // - Vertical wall (x constant): if direction is bottom->top, "left" is -X (west).
                    var r1 = hit1.Value;
                    var r2 = hit2.Value;

                    bool r1OnFaceA = IsRoomOnFaceA(r1, a, b, vertical, horizontal);
                    bool r2OnFaceA = IsRoomOnFaceA(r2, a, b, vertical, horizontal);

                    // If both evaluate the same due to identical placement bug, fall back to ID ordering.
                    if (r1OnFaceA == r2OnFaceA)
                    {
                        if (string.CompareOrdinal(r1.Room.Id.Value, r2.Room.Id.Value) <= 0)
                        {
                            wall.RoomAId = r1.Room.Id;
                            wall.RoomBId = r2.Room.Id;
                        }
                        else
                        {
                            wall.RoomAId = r2.Room.Id;
                            wall.RoomBId = r1.Room.Id;
                        }
                    }
                    else
                    {
                        wall.RoomAId = r1OnFaceA ? r1.Room.Id : r2.Room.Id;
                        wall.RoomBId = r1OnFaceA ? r2.Room.Id : r1.Room.Id;
                    }
                }

                // Update each room's RoomWallRef.InteriorFace
                UpdateRoomWallInteriorFaces(graph, wall, junctionPos);
            }
        }

        private static bool RoomTouchesWall(RoomPlacementRuntime rp, Vector2Int a, Vector2Int b, bool vertical, bool horizontal)
        {
            // Normalize segment bounds
            var min = new Vector2Int(Math.Min(a.x, b.x), Math.Min(a.y, b.y));
            var max = new Vector2Int(Math.Max(a.x, b.x), Math.Max(a.y, b.y));

            // Room rectangle bounds
            var rMin = rp.Min;
            var rMax = rp.Max;

            if (horizontal)
            {
                // Wall at y = a.y spanning x in [min.x, max.x]
                // Room touches it if it matches exactly its North or South edge and overlaps segment fully.
                bool onSouthEdge = a.y == rMin.y;
                bool onNorthEdge = a.y == rMax.y;
                if (!onSouthEdge && !onNorthEdge) return false;

                // Segment must be within that edge span of the room
                return min.x >= rMin.x && max.x <= rMax.x;
            }
            else // vertical
            {
                // Wall at x = a.x spanning y in [min.y, max.y]
                bool onWestEdge = a.x == rMin.x;
                bool onEastEdge = a.x == rMax.x;
                if (!onWestEdge && !onEastEdge) return false;

                return min.y >= rMin.y && max.y <= rMax.y;
            }
        }

        private static bool IsRoomOnFaceA(RoomPlacementRuntime rp, Vector2Int a, Vector2Int b, bool vertical, bool horizontal)
        {
            // FaceA is "left side" when traveling Start->End.
            // We'll pick a test point slightly off the wall toward FaceA, and see if it's inside the room rect.
            // Offset is 1 U16 (1/16") which is safe on your grid.
            const int eps = 1;

            var mid = new Vector2Int((a.x + b.x) / 2, (a.y + b.y) / 2);

            Vector2Int probe;
            if (horizontal)
            {
                // Determine direction
                bool leftToRight = b.x > a.x;
                // Left of direction is +Y if left->right, else -Y
                probe = leftToRight ? new Vector2Int(mid.x, mid.y + eps) : new Vector2Int(mid.x, mid.y - eps);
            }
            else
            {
                bool bottomToTop = b.y > a.y;
                // Left of direction is -X if bottom->top, else +X
                probe = bottomToTop ? new Vector2Int(mid.x - eps, mid.y) : new Vector2Int(mid.x + eps, mid.y);
            }

            return PointInRect(probe, rp.Min, rp.Max);
        }

        private static bool PointInRect(Vector2Int p, Vector2Int min, Vector2Int max)
        {
            // Half-open interval is fine for probes with eps
            return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y;
        }

        private static void UpdateRoomWallInteriorFaces(
            BuildingGraph graph,
            Wall wall,
            Dictionary<string, Vector2Int> junctionPos)
        {
            // For each room that references this wall, set InteriorFace to FaceA if that room is RoomAId, else FaceB.
            void Apply(Room room)
            {
                for (int i = 0; i < room.BoundaryWalls.Count; i++)
                {
                    if (!room.BoundaryWalls[i].WallId.Equals(wall.Id)) continue;

                    var rwr = room.BoundaryWalls[i];
                    if (room.Id.Equals(wall.RoomAId)) rwr.InteriorFace = WallFace.FaceA;
                    else if (room.Id.Equals(wall.RoomBId)) rwr.InteriorFace = WallFace.FaceB;
                    // Exterior: if only RoomA set, interior is FaceA by definition
                    else if (wall.Kind == WallKind.Exterior && room.Id.Equals(wall.RoomAId)) rwr.InteriorFace = WallFace.FaceA;

                    room.BoundaryWalls[i] = rwr;
                }
            }

            if (!string.IsNullOrEmpty(wall.RoomAId.Value))
            {
                var ra = graph.FindRoom(wall.RoomAId);
                if (ra != null) Apply(ra);
            }
            if (!string.IsNullOrEmpty(wall.RoomBId.Value))
            {
                var rb = graph.FindRoom(wall.RoomBId);
                if (rb != null) Apply(rb);
            }
        }

        // -------------------------
        // Junction connectivity rebuild
        // -------------------------

        private static void RebuildJunctionConnectivity(BuildingGraph graph)
        {
            // Clear
            foreach (var j in graph.Junctions)
            {
                if (j.ConnectedWallIds == null) j.ConnectedWallIds = new List<EntityId>();
                else j.ConnectedWallIds.Clear();
            }

            // Index junctions by ID
            var jById = new Dictionary<string, Junction>();
            foreach (var j in graph.Junctions) jById[j.Id.Value] = j;

            foreach (var w in graph.Walls)
            {
                if (jById.TryGetValue(w.StartJunctionId.Value, out var js))
                    js.ConnectedWallIds.Add(w.Id);

                if (jById.TryGetValue(w.EndJunctionId.Value, out var je))
                    je.ConnectedWallIds.Add(w.Id);
            }

            // Optionally update junction type based on degree
            foreach (var j in graph.Junctions)
            {
                int degree = j.ConnectedWallIds.Count;
                j.Type = degree switch
                {
                    0 => JunctionType.Custom,
                    1 => JunctionType.End,
                    2 => JunctionType.CornerL, // could also be straight; left simple for V1
                    3 => JunctionType.Tee,
                    4 => JunctionType.Cross,
                    _ => JunctionType.Custom
                };
            }
        }
    }
}

