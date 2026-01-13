
import math

class GeoUtils:
    WGS84_A = 6378137.0
    WGS84_B = 6356752.314245
    WGS84_E2 = 1 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A)

    @staticmethod
    def meters_per_lat_degree(lat: float) -> float:
        lat_rad = math.radians(lat)
        sin_lat = math.sin(lat_rad)
        numerator = math.pi * GeoUtils.WGS84_A * (1 - GeoUtils.WGS84_E2)
        denominator = 180 * math.pow(1 - GeoUtils.WGS84_E2 * sin_lat * sin_lat, 1.5)
        return numerator / denominator
    
    @staticmethod
    def meters_per_lon_degree(lat: float) -> float:
        lat_rad = math.radians(lat)
        cos_lat = math.cos(lat_rad)
        sin_lat = math.sin(lat_rad)
        numerator = math.pi * GeoUtils.WGS84_A * cos_lat
        denominator = 180 * math.sqrt(1 - GeoUtils.WGS84_E2 * sin_lat * sin_lat)
        return numerator / denominator

    @staticmethod
    def xyz_to_gps(x: float, y: float, z: float, origin_lat: float, origin_lon: float, origin_alt: float) -> dict:
        meters_per_lat = GeoUtils.meters_per_lat_degree(origin_lat)
        dlat = y / meters_per_lat
        lat = origin_lat + dlat
        mid_lat = (origin_lat + lat) / 2.0
        meters_per_lon = GeoUtils.meters_per_lon_degree(mid_lat)
        dlon = x / meters_per_lon
        lon = origin_lon + dlon
        return {'lat': lat, 'lon': lon}

def test_precision():
    # 模拟深圳某地
    origin_lat = 22.500000000
    origin_lon = 113.900000000
    
    print(f"原点: {origin_lat:.9f}, {origin_lon:.9f}")
    
    # 1厘米的位移
    x = 0.01 
    y = 0.01 
    
    res = GeoUtils.xyz_to_gps(x, y, 0, origin_lat, origin_lon, 0)
    
    print(f"偏移1cm后的坐标: {res['lat']:.12f}, {res['lon']:.12f}")
    
    # 逆向验证
    # 纬度差
    dlat = res['lat'] - origin_lat
    # 经度差
    dlon = res['lon'] - origin_lon
    
    m_lat = GeoUtils.meters_per_lat_degree(origin_lat)
    m_lon = GeoUtils.meters_per_lon_degree((origin_lat + res['lat'])/2)
    
    calc_y = dlat * m_lat
    calc_x = dlon * m_lon
    
    print(f"反算回来的Y (应为0.01): {calc_y:.12f}")
    print(f"反算回来的X (应为0.01): {calc_x:.12f}")
    print(f"Y误差: {abs(calc_y - y):.12e} 米")
    print(f"X误差: {abs(calc_x - x):.12e} 米")
    
if __name__ == "__main__":
    test_precision()
