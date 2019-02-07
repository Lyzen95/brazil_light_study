import gdal, ogr, osr
import numpy as np
import sys
import os
import matplotlib.pyplot as plt

# class to handle GeoTif using gdal/ogr
# Reference: https://pcjericks.github.io/py-gdalogr-cookbook/index.html
class GeoTif:
    def __init__(self, tifFile, noData=255):
        self.gtif = gdal.Open(tifFile, gdal.GA_ReadOnly)
        transform = self.gtif.GetGeoTransform()
        self.spatialRef = osr.SpatialReference()
        self.spatialRef.ImportFromWkt(self.gtif.GetProjectionRef())
        self.xOrigin = transform[0]
        self.yOrigin = transform[3]
        self.pixelWidth = transform[1]
        self.pixelHeight = transform[5]
        self.noData = noData

    def getInnerGeometry(self, geometry):
        #Compute pixels to take from image with geometry bbox
        minX, minY, maxX, maxY = getGeometryBbox(geometry)
        xoff = int((minX - self.xOrigin)/self.pixelWidth)
        yoff = int((self.yOrigin - maxY)/self.pixelWidth)
        xcount = int((maxX - minX)/self.pixelWidth)+1
        ycount = int((maxY - minY)/self.pixelWidth)+1

        # Create memory target raster to store result
        memRaster = gdal.GetDriverByName('MEM').Create('', xcount, ycount, 1, gdal.GDT_Byte)
        memRaster.SetGeoTransform((
            minX, self.pixelWidth, 0,
            maxY, 0, self.pixelHeight,
        ))
        rasterSR = osr.SpatialReference()
        rasterSR.ImportFromWkt(self.gtif.GetProjectionRef())
        memRaster.SetProjection(rasterSR.ExportToWkt())

        # create temporary file to get layer from geometry
        outDriver = ogr.GetDriverByName("ESRI Shapefile")
        outDataSource = outDriver.CreateDataSource("tmp.shp")
        outLayer = outDataSource.CreateLayer("./", geom_type=geometry.GetGeometryType())
        featureDefn = outLayer.GetLayerDefn()
        feature = ogr.Feature(featureDefn)
        feature.SetGeometry(geometry)
        outLayer.CreateFeature(feature)

        # create mask from layer
        gdal.RasterizeLayer(memRaster, [1], outLayer, burn_values=[1])
        band = self.gtif.GetRasterBand(1)
        dataRaster = band.ReadAsArray(xoff, yoff, xcount, ycount).astype(np.float)
        bandMask = memRaster.GetRasterBand(1)
        dataMask = bandMask.ReadAsArray(0, 0, xcount, ycount).astype(np.float)

        # Mask zone of raster
        zoneraster = np.ma.masked_array(dataRaster,  np.logical_not(dataMask))

        # Delete temporary file
        outDriver.DeleteDataSource("tmp.shp")

        return zoneraster

# return geometries dictionnary (name => gdal.geometry) from a directory
def geometryDict(directory):
    geomDict = {}
    for root, dirs, files in os.walk(directory):
        for f in files:
            with open(directory+f) as file:
                geomDict[f.split(".")[0]] = ogr.CreateGeometryFromWkt(file.read())
    return geomDict

# return the bounding box of a geometry
# (minX, minY, maxX, maxY)
def getGeometryBbox(geometry):
    if (geometry.GetGeometryName() == 'MULTIPOLYGON'):
        count = 0
        pointsX = []; pointsY = []
        for polygon in geometry:
            geomInner = geometry.GetGeometryRef(count)
            ring = geomInner.GetGeometryRef(0)
            numpoints = ring.GetPointCount()
        for p in range(numpoints):
            lon, lat, z = ring.GetPoint(p)
            pointsX.append(lon)
            pointsY.append(lat)
            count += 1
    elif (geometry.GetGeometryName() == 'POLYGON'):
        ring = geometry.GetGeometryRef(0)
        numpoints = ring.GetPointCount()
        pointsX = []; pointsY = []
        for p in range(numpoints):
            lon, lat, z = ring.GetPoint(p)
            pointsX.append(lon)
            pointsY.append(lat)
    else:
        print("ERROR: Geometry type %s isn't handle"%geometry.GetGeometryName())
        exit()
    return (min(pointsX), min(pointsY), max(pointsX), max(pointsY))

def uncompressMaps():
    os.system("gzip -d ../maps/*.tif.gz")

def compressMaps():
    os.system("gzip ../maps/*.tif")

def filterDataValue(v):
    if v < 64:
        return v
    else:
        return 0

header = ["zone", "light_sum", "year"]
content = []
print("Loading zones geometries...")
geomDict = geometryDict("../zones/")
print("Geometries loaded\n\n")
for root, dirs, files in os.walk("../maps/"):
    for f in files:
        if f.split(".")[-1] == "tif":
            tif = GeoTif("../maps/"+f)
            year = f.split(".")[0][3:]
            print("Treating year %s...\n"%year)
            for zone in geomDict.keys():
                print("Treating zone %s"%zone)
                zonePix = tif.getInnerGeometry(geomDict[zone])
                if zone == "brazil":
                    print("Creating snapfile ../brazil_snaps/%s_brazil_light"%year)
                    fig = plt.figure()
                    plt.imshow(zonePix)
                    fig.savefig("../brazil_snaps/%s_brazil_light.png"%year)
                    print("Snap created")
                print("Computing zone stat...")
                vectFunc = np.vectorize(filterDataValue,otypes=[np.float],cache=False)
                content += [[zone, np.sum(vectFunc(zonePix)), year]]
                print("Zone %s treated\n\n"%zone)
            print("Year %s treated\n\n"%year)
print("Writing results to file results.csv...")
with open("../results.csv","w") as r:
    lines = []
    lines += ",".join(header)+"\n"
    for l in content:
        l = map(str, l)
        lines += ",".join(l)+"\n"
    r.writelines(lines)
