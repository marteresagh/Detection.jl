using Detection
using Visualization
using Common
using FileManager


Voriginal,VV,rgb = PointClouds.loadlas("examples/PAPER/planimetriaMERGE_planexy.las")
Voriginal,VV,rgb = PointClouds.loadlas("examples/PAPER/muriAngolo.las")
# Voriginal,VV,rgb = PointClouds.loadlas("examples/PAPER/planimetria_planexy_subsample.las")
trasl,Vtrasl = PointClouds.subtractaverage(Voriginal)
V = Vtrasl[1:2,:]
PC = PointClouds.PointCloud(size(V,2),V,rgb)

GL.VIEW(
	[
		GL.GLPoints(convert(Lar.Points,V'))
		#GL.GLGrid(L,EL)
		#GL.GLAxis(GL.Point3d(0,0,0),GL.Point3d(1,1,1))
	]
);


par = 0.06
threshold = 2*0.03
@time LINES = PointClouds.LinesDetectionRandom(PC, 0.06, 2*0.03, 400, 100)
linedetected = PointClouds.LineDetectionFromRandomInitPoint(PC,par,threshold)
L,EL = PointClouds.DrawLines(LINES,0.0)
ViewLines(LINES)
ViewLines(LINES2)


W = nonpresi(V,LINES)
GL.VIEW(
	[

		#GL.GLPoints(convert(Lar.Points,V'))
		GL.GLPoints(convert(Lar.Points,W'),GL.COLORS[2])
		GL.GLGrid(L,EL,GL.COLORS[1],1.0)
		#GL.GLPoints(convert(Lar.Points,pointsonline.points'))
		#GL.GLAxis(GL.Point3d(0,0,0),GL.Point3d(1,1,1))
	]
);




################## 3 SEZIONI
# header,laspoints = PointClouds.load("examples/PAPER/MERGE_5cm.las")
# pvec = PointClouds.set_z_zero(laspoints,header)
# PointClouds.LasIO.update!(header,pvec)
# using LasIO
# LasIO.FileIO.save("examples/PAPER/MERGE_5cm_floor.las",header,pvec) # file poi decimato con CC
#


#####################  1 cm
Voriginal,VV,rgb = PointClouds.loadlas("examples/PAPER/MERGE_1cm_floor.las")
trasl,Vtrasl = PointClouds.subtractaverage(Voriginal)
V = Vtrasl[1:2,:]
PC = PointClouds.PointCloud(size(V,2),V,rgb)

GL.VIEW(
	[
		GL.GLPoints(convert(Lar.Points,V'))
	]
);

@time LINES = PointClouds.LinesDetectionRandom(PC, 0.06, 3*0.03, 1000, 100)
L,EL = PointClouds.DrawLines(LINES,0.0)

ViewLines(LINES)

W = nonpresi(V,LINES)
GL.VIEW(
	[

		#GL.GLPoints(convert(Lar.Points,V'))
		GL.GLPoints(convert(Lar.Points,W'),GL.COLORS[2])
		GL.GLGrid(L,EL,GL.COLORS[1],1.0)
		#GL.GLPoints(convert(Lar.Points,pointsonline.points'))
		#GL.GLAxis(GL.Point3d(0,0,0),GL.Point3d(1,1,1))
	]
);


#####################  3 cm
Voriginal,VV,rgb = PointClouds.loadlas("examples/PAPER/MERGE_3cm_floor.las")
trasl,Vtrasl = PointClouds.subtractaverage(Voriginal)
V2 = Vtrasl[1:2,:]
PC2 = PointClouds.PointCloud(size(V2,2),V2,rgb)

GL.VIEW(
	[
		GL.GLPoints(convert(Lar.Points,V2'))
	]
);
@time LINES2 = PointClouds.LinesDetectionRandom(PC2, 0.06, 2*0.03, 1000, 100)
L2,EL2 = PointClouds.DrawLines(LINES2,0.0)

ViewLines(LINES2)

W2 = nonpresi(V2,LINES2)
GL.VIEW(
	[

		#GL.GLPoints(convert(Lar.Points,V2'))
		GL.GLPoints(convert(Lar.Points,W2'),GL.COLORS[2])
		GL.GLGrid(L2,EL2,GL.COLORS[1],1.0)
		#GL.GLPoints(convert(Lar.Points,pointsonline.points'))
		#GL.GLAxis(GL.Point3d(0,0,0),GL.Point3d(1,1,1))
	]
);


#####################  5 cm
Voriginal,VV,rgb = PointClouds.loadlas("examples/PAPER/MERGE_5cm_floor.las")
trasl,Vtrasl = PointClouds.subtractaverage(Voriginal)
V3 = Vtrasl[1:2,:]
PC3 = PointClouds.PointCloud(size(V3,2),V3,rgb)

GL.VIEW(
	[
		GL.GLPoints(convert(Lar.Points,V3'))
	]
);
@time LINES3 = PointClouds.LinesDetectionRandom(PC3, 0.06, 2*0.03, 1000, 100)
L3,EL3 = PointClouds.DrawLines(LINES3,0.0)

ViewLines(LINES3)

W3 = nonpresi(V3,LINES3)
T3 = presi(V3,LINES3)
W3D = vcat(W3,(-3.0.*ones(size(W3,2)))')
T3D = vcat(T3,(-3.0.*ones(size(T3,2)))')
GL.VIEW(
	[

		#GL.GLPoints(convert(Lar.Points,V2'))
		GL.GLPoints(convert(Lar.Points,W3'),GL.COLORS[2])
		GL.GLGrid(L3,EL3,GL.COLORS[1],1.0)
		#GL.GLPoints(convert(Lar.Points,pointsonline.points'))
		#GL.GLAxis(GL.Point3d(0,0,0),GL.Point3d(1,1,1))
	]
);
