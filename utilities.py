from pyproj import Proj, transform

def cvt_gll_ddmm_2_dd (st):
    ilat = st[0]
    ilon = st[2]
    olat = float(int(ilat/100))
    olon = float(int(ilon/100))
    olat_mm = (ilat%100)/60
    olon_mm = (ilon%100)/60
    olat += olat_mm
    olon += olon_mm
    if st[3] == "W":
        olon = -olon
    return olat,olon

def projDegree2Meter(lon, lat):
    projDegree2Meter = Proj("+proj=utm +zone=30, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    x,y = projDegree2Meter(lon, lat) # convert to meters
    return x, y

