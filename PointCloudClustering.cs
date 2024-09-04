using System;
using System.Collections.Generic;
using System.IO;
using NXOpen;

namespace WednesdayTaskNX
{
    public class PointCloudClustering
    {
        public static void Main()
        {
            try
            {
                // Get the NX session
                Session session = Session.GetSession();
                if (session == null)
                {
                    Console.WriteLine("Failed to get NX session.");
                    return;
                }

                // Get the active part
                Part workPart = session.Parts.Work;
                if (workPart == null)
                {
                    session.ListingWindow.WriteLine("No active part found. Please open a part.");
                    return;
                }

                // Load the point cloud data
                string filePath = @"C:\Users\diksh\Desktop\open3d_data\download\BunnyMesh\BunnyMesh.ply";
                List<NXOpen.Point3d> points = LoadPointCloud(filePath);

                // Check if points are loaded
                if (points.Count == 0)
                {
                    session.ListingWindow.WriteLine("No points loaded from file.");
                    return;
                }

                session.ListingWindow.WriteLine("Loaded " + points.Count + " points from file.");

                // Visualize the point cloud
                VisualizePointCloud(workPart, points);

                // Calculate and visualize bounding box
                BoundingBox boundingBox = CalculateBoundingBox(points);
                VisualizeBoundingBox(workPart, boundingBox);

                // Cluster points using custom DBSCAN
                List<List<NXOpen.Point3d>> clusters = ClusterPointCloud(points);
                session.ListingWindow.WriteLine("Generated " + clusters.Count + " clusters.");

                // Optional: Generate mesh from clustered points (not implemented)
                // GenerateMeshFromClusters(workPart, clusters);

                // Keep NX Open running
                session.ListingWindow.Open();
                session.ListingWindow.WriteLine("Point cloud processing complete. Press any key to exit.");
                Console.ReadKey();
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error: " + ex.Message);
            }
        }

        static List<NXOpen.Point3d> LoadPointCloud(string filePath)
        {
            List<NXOpen.Point3d> points = new List<NXOpen.Point3d>();
            bool headerParsed = false;
            int vertexCount = 0;

            try
            {
                using (StreamReader reader = new StreamReader(filePath))
                {
                    string line;
                    while ((line = reader.ReadLine()) != null)
                    {
                        if (line.StartsWith("element vertex"))
                        {
                            var parts = line.Split(' ');
                            if (parts.Length == 3)
                            {
                                vertexCount = int.Parse(parts[2]);
                            }
                        }
                        else if (line.StartsWith("end_header"))
                        {
                            headerParsed = true;
                            continue;
                        }

                        if (headerParsed)
                        {
                            var parts = line.Split(' ');
                            if (parts.Length >= 3 && vertexCount > 0)
                            {
                                double x = double.Parse(parts[0]);
                                double y = double.Parse(parts[1]);
                                double z = double.Parse(parts[2]);
                                points.Add(new NXOpen.Point3d(x, y, z));
                                vertexCount--;
                                if (vertexCount <= 0) break;
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error reading point cloud file: " + ex.Message);
            }

            return points;
        }

        static void VisualizePointCloud(Part workPart, List<NXOpen.Point3d> points)
        {
            foreach (var point in points)
            {
                try
                {
                    Point nxPoint = workPart.Points.CreatePoint(point);
                    nxPoint.SetVisibility(SmartObject.VisibilityOption.Visible);
                }
                catch (Exception ex)
                {
                    Session.GetSession().ListingWindow.WriteLine("Error creating point: " + ex.Message);
                }
            }
        }

        static BoundingBox CalculateBoundingBox(List<NXOpen.Point3d> points)
        {
            double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue;
            double maxX = double.MinValue, maxY = double.MinValue, maxZ = double.MinValue;

            foreach (var point in points)
            {
                if (point.X < minX) minX = point.X;
                if (point.Y < minY) minY = point.Y;
                if (point.Z < minZ) minZ = point.Z;

                if (point.X > maxX) maxX = point.X;
                if (point.Y > maxY) maxY = point.Y;
                if (point.Z > maxZ) maxZ = point.Z;
            }

            return new BoundingBox(new NXOpen.Point3d(minX, minY, minZ), new NXOpen.Point3d(maxX, maxY, maxZ));
        }

        static void VisualizeBoundingBox(Part workPart, BoundingBox boundingBox)
        {
            List<NXOpen.Point3d> boxCorners = new List<NXOpen.Point3d>
            {
                boundingBox.Min,
                new NXOpen.Point3d(boundingBox.Max.X, boundingBox.Min.Y, boundingBox.Min.Z),
                new NXOpen.Point3d(boundingBox.Max.X, boundingBox.Max.Y, boundingBox.Min.Z),
                new NXOpen.Point3d(boundingBox.Min.X, boundingBox.Max.Y, boundingBox.Min.Z),
                new NXOpen.Point3d(boundingBox.Min.X, boundingBox.Min.Y, boundingBox.Max.Z),
                new NXOpen.Point3d(boundingBox.Max.X, boundingBox.Min.Y, boundingBox.Max.Z),
                new NXOpen.Point3d(boundingBox.Max.X, boundingBox.Max.Y, boundingBox.Max.Z),
                new NXOpen.Point3d(boundingBox.Min.X, boundingBox.Max.Y, boundingBox.Max.Z)
            };

            for (int i = 0; i < 4; i++)
            {
                CreateLine(workPart, boxCorners[i], boxCorners[(i + 1) % 4]);
                CreateLine(workPart, boxCorners[i + 4], boxCorners[(i + 1) % 4 + 4]);
                CreateLine(workPart, boxCorners[i], boxCorners[i + 4]);
            }
        }

        static void CreateLine(Part workPart, NXOpen.Point3d start, NXOpen.Point3d end)
        {
            Line line = workPart.Curves.CreateLine(start, end);
            line.SetVisibility(SmartObject.VisibilityOption.Visible);
        }

        static List<List<NXOpen.Point3d>> ClusterPointCloud(List<NXOpen.Point3d> points)
        {
            // Convert points to double array format
            double[][] pointArray = new double[points.Count][];
            for (int i = 0; i < points.Count; i++)
            {
                pointArray[i] = new double[] { points[i].X, points[i].Y, points[i].Z };
            }

            // Perform DBSCAN clustering
            DBSCAN dbscan = new DBSCAN(0.1, 10); // Epsilon and MinPoints
            int[] labels = dbscan.Cluster(pointArray);

            // Group points by cluster
            var clusters = new Dictionary<int, List<NXOpen.Point3d>>();
            for (int i = 0; i < labels.Length; i++)
            {
                int label = labels[i];
                if (!clusters.ContainsKey(label))
                {
                    clusters[label] = new List<NXOpen.Point3d>();
                }
                clusters[label].Add(points[i]);
            }

            return new List<List<NXOpen.Point3d>>(clusters.Values);
        }
    }

    public class BoundingBox
    {
        public NXOpen.Point3d Min { get; private set; }
        public NXOpen.Point3d Max { get; private set; }

        public BoundingBox(NXOpen.Point3d min, NXOpen.Point3d max)
        {
            Min = min;
            Max = max;
        }
    }

    // Custom DBSCAN implementation
    public class DBSCAN
    {
        private double epsilon;
        private int minPoints;

        public DBSCAN(double epsilon, int minPoints)
        {
            this.epsilon = epsilon;
            this.minPoints = minPoints;
        }

        public int[] Cluster(double[][] points)
        {
            int n = points.Length;
            int[] labels = new int[n];
            int currentCluster = 0;

            for (int i = 0; i < n; i++)
            {
                if (labels[i] != 0) continue;

                List<int> neighbors = RegionQuery(points, i);
                if (neighbors.Count < minPoints)
                {
                    labels[i] = -1; // Noise
                }
                else
                {
                    currentCluster++;
                    ExpandCluster(points, labels, i, neighbors, currentCluster);
                }
            }

            return labels;
        }

        private List<int> RegionQuery(double[][] points, int index)
        {
            List<int> neighbors = new List<int>();
            for (int i = 0; i < points.Length; i++)
            {
                if (Distance(points[index], points[i]) <= epsilon)
                {
                    neighbors.Add(i);
                }
            }
            return neighbors;
        }

        private void ExpandCluster(double[][] points, int[] labels, int index, List<int> neighbors, int clusterId)
        {
            labels[index] = clusterId;

            int i = 0;
            while (i < neighbors.Count)
            {
                int neighborIndex = neighbors[i];
                if (labels[neighborIndex] == 0)
                {
                    List<int> newNeighbors = RegionQuery(points, neighborIndex);
                    if (newNeighbors.Count >= minPoints)
                    {
                        neighbors.AddRange(newNeighbors);
                    }
                }
                if (labels[neighborIndex] == -1)
                {
                    labels[neighborIndex] = clusterId;
                }
                i++;
            }
        }

        private double Distance(double[] point1, double[] point2)
        {
            double sum = 0.0;
            for (int i = 0; i < point1.Length; i++)
            {
                sum += (point1[i] - point2[i]) * (point1[i] - point2[i]);
            }
            return Math.Sqrt(sum);
        }
    }
}
