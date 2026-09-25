# -*- coding: utf-8 -*-
"""
螺栓类型定义、参数 schema 与标准尺寸库（预设值为联网核对的常用标准值）。

本模块不依赖任何第三方库（纯 Python），可被 CATIA 控制器与 UI 共用，
也可在离线环境单独 import 做几何/参数校验。
参考：文件夹内 readme.md《螺栓全面技术指南》。

覆盖范围：
  * 全部螺栓种类（六角/法兰/内六角/方头/圆头/沉头/半沉头/马车/吊环/双头螺柱/翼形等 15 类）
  * 全部螺纹体系：公制粗牙/细牙/超细牙、英制 UNC/UNF/UNRF、英制 BSW(55°)
  * 全部精细程度（螺纹公差带）：4g/5g/6e/6f/6g/6h/8g/10g
  * 产品等级：A/B/C；性能等级：4.6~12.9

尺寸来源（均为公开标准常用值，建模用“公称/最大值”，单位 mm）：
  hex_full      GB/T 5782-2016  六角头螺栓 全螺纹（=ISO 4014 / DIN 933）
  hex_partial   GB/T 5783-2016  六角头螺栓 部分螺纹（=ISO 4017 / DIN 931）
  hex_flange    GB/T 5787.2-2016 六角法兰面螺栓（=DIN 6921 / ISO 10667）
  socket_cap    GB/T 70.1-2008  内六角圆柱头螺钉（=ISO 4762 / DIN 912）
  socket_pan    GB/T 70.3-2008  内六角沉头螺钉（=ISO 4758 / DIN 915）
  socket_dome   GB/T 70.4-2008  内六角杯头螺钉（=ISO 4759 / DIN 917）
  socket_hex    GB/T 70.2-2008  内六角全六角螺钉（近似）
  square_head   GB/T 5787-2016  方头螺栓（C级，近似 ISO 4028）
  button_head   GB/T 818-2008  十字槽/圆柱头螺钉近似（=ISO 4023 / DIN 404）
  countersunk   GB/T 5780-2016  沉头螺栓（=ISO 4030 / DIN 617）
  pan_head      GB/T 5785-2016  半沉头螺栓（近似）
  carriage      GB/T 5781-2016  马车螺栓（方颈，近似）
  eye_bolt      GB/T 823-2009  吊环螺栓（近似）
  stud          GB/T 896-2010  双头螺柱（=ISO 899 / DIN 976）
  wing_bolt     GB/T 25159-2010 蝶形螺栓（近似）
注：螺距 P 取粗牙（圆螺母取标准细牙）。实际生产有公差，建模以公称尺寸为准。
"""
import math
import re

# 大类分组（供 UI 树形展示）
BOLT_CATEGORIES = [
    ("hex", "六角 / 方形头螺栓"),
    ("socket", "内六角类螺栓"),
    ("round", "圆 / 沉头类螺栓"),
    ("special", "特殊螺栓"),
]

# ---------------- 螺纹标准数据库 ----------------
# 覆盖：公制粗牙 / 公制细牙 / 公制超细牙 / 英制 UNC(粗) / UNF(细) / UNRF(超细) / BSW(惠氏,55°)。
# 每个尺寸元组: (公称大径 mm, 螺距 mm)。英制由 TPI 换算: 螺距 = 25.4 / TPI。
# 牙型角: 公制/UN 系列 60°；BSW(惠氏) 55°。
THREAD_SYSTEMS = {
    "metric_coarse": {
        "name": "公制 粗牙 (M, Coarse)", "angle": 60, "unit": "mm",
        "sizes": {
            "M1.6": (1.6, 0.35), "M2": (2.0, 0.4), "M2.5": (2.5, 0.45),
            "M3": (3.0, 0.5), "M3.5": (3.5, 0.6), "M4": (4.0, 0.7),
            "M5": (5.0, 0.8), "M6": (6.0, 1.0), "M8": (8.0, 1.25),
            "M10": (10.0, 1.5), "M12": (12.0, 1.75), "M14": (14.0, 2.0),
            "M16": (16.0, 2.0), "M18": (18.0, 2.5), "M20": (20.0, 2.5),
            "M22": (22.0, 2.5), "M24": (24.0, 3.0), "M27": (27.0, 3.0),
            "M30": (30.0, 3.5), "M33": (33.0, 3.5), "M36": (36.0, 4.0),
            "M39": (39.0, 4.0), "M42": (42.0, 4.5), "M45": (45.0, 4.5),
            "M48": (48.0, 5.0),
        },
    },
    "metric_fine": {
        "name": "公制 细牙 (M, Fine)", "angle": 60, "unit": "mm",
        "sizes": {
            "M3×0.5": (3.0, 0.5), "M4×0.5": (4.0, 0.5), "M5×0.5": (5.0, 0.5),
            "M6×0.75": (6.0, 0.75), "M8×1": (8.0, 1.0), "M8×0.75": (8.0, 0.75),
            "M10×1": (10.0, 1.0), "M10×1.25": (10.0, 1.25), "M12×1.25": (12.0, 1.25),
            "M12×1.5": (12.0, 1.5), "M14×1.5": (14.0, 1.5), "M16×1.5": (16.0, 1.5),
            "M18×1.5": (18.0, 1.5), "M20×1.5": (20.0, 1.5), "M22×1.5": (22.0, 1.5),
            "M24×2": (24.0, 2.0), "M24×1.5": (24.0, 1.5), "M27×2": (27.0, 2.0),
            "M30×2": (30.0, 2.0), "M33×2": (33.0, 2.0), "M36×3": (36.0, 3.0),
            "M42×3": (42.0, 3.0), "M48×3": (48.0, 3.0),
        },
    },
    "metric_extra_fine": {
        "name": "公制 超细牙 (M, Extra Fine)", "angle": 60, "unit": "mm",
        "sizes": {
            "M6×0.5": (6.0, 0.5), "M8×0.5": (8.0, 0.5), "M10×0.5": (10.0, 0.5),
            "M10×0.75": (10.0, 0.75), "M12×0.5": (12.0, 0.5), "M12×0.75": (12.0, 0.75),
            "M14×1": (14.0, 1.0), "M16×1": (16.0, 1.0), "M18×1": (18.0, 1.0),
            "M20×1": (20.0, 1.0), "M22×1": (22.0, 1.0), "M24×1.5": (24.0, 1.5),
            "M27×1.5": (27.0, 1.5), "M30×1.5": (30.0, 1.5), "M33×1.5": (33.0, 1.5),
            "M36×1.5": (36.0, 1.5), "M42×2": (42.0, 2.0), "M48×2": (48.0, 2.0),
        },
    },
    "unc": {
        "name": "英制 UNC (粗牙)", "angle": 60, "unit": "inch",
        "sizes": {
            '1/8"-40': (3.175, 25.4 / 40), '3/16"-24': (4.763, 25.4 / 24),
            '1/4"-20': (6.35, 25.4 / 20), '5/16"-18': (7.938, 25.4 / 18),
            '3/8"-16': (9.525, 25.4 / 16), '7/16"-14': (11.112, 25.4 / 14),
            '1/2"-13': (12.7, 25.4 / 13), '9/16"-12': (14.288, 25.4 / 12),
            '5/8"-11': (15.875, 25.4 / 11), '3/4"-10': (19.05, 25.4 / 10),
            '7/8"-9': (22.225, 25.4 / 9), '1"-8': (25.4, 25.4 / 8),
            '1 1/8"-7': (28.575, 25.4 / 7), '1 1/4"-7': (31.75, 25.4 / 7),
            '1 1/2"-6': (38.1, 25.4 / 6), '2"-4.5': (50.8, 25.4 / 4.5),
        },
    },
    "unf": {
        "name": "英制 UNF (细牙)", "angle": 60, "unit": "inch",
        "sizes": {
            '1/8"-44': (3.175, 25.4 / 44), '3/16"-32': (4.763, 25.4 / 32),
            '1/4"-28': (6.35, 25.4 / 28), '5/16"-24': (7.938, 25.4 / 24),
            '3/8"-24': (9.525, 25.4 / 24), '7/16"-20': (11.112, 25.4 / 20),
            '1/2"-20': (12.7, 25.4 / 20), '9/16"-18': (14.288, 25.4 / 18),
            '5/8"-18': (15.875, 25.4 / 18), '3/4"-16': (19.05, 25.4 / 16),
            '7/8"-14': (22.225, 25.4 / 14), '1"-12': (25.4, 25.4 / 12),
            '1 1/8"-12': (28.575, 25.4 / 12), '1 1/4"-12': (31.75, 25.4 / 12),
            '1 1/2"-12': (38.1, 25.4 / 12), '2"-12': (50.8, 25.4 / 12),
        },
    },
    "unrf": {
        "name": "英制 UNRF (超细牙)", "angle": 60, "unit": "inch",
        "sizes": {
            '1/4"-36': (6.35, 25.4 / 36), '5/16"-32': (7.938, 25.4 / 32),
            '3/8"-32': (9.525, 25.4 / 32), '7/16"-28': (11.112, 25.4 / 28),
            '1/2"-28': (12.7, 25.4 / 28), '9/16"-24': (14.288, 25.4 / 24),
            '5/8"-24': (15.875, 25.4 / 24), '3/4"-24': (19.05, 25.4 / 24),
            '7/8"-24': (22.225, 25.4 / 24), '1"-20': (25.4, 25.4 / 20),
            '1 1/2"-20': (38.1, 25.4 / 20), '2"-20': (50.8, 25.4 / 20),
        },
    },
    "bsw": {
        "name": "英制 BSW (惠氏 粗牙, 55°)", "angle": 55, "unit": "inch",
        "sizes": {
            '1/8"': (3.175, 25.4 / 40), '3/16"': (4.763, 25.4 / 24),
            '1/4"': (6.35, 25.4 / 20), '5/16"': (7.938, 25.4 / 18),
            '3/8"': (9.525, 25.4 / 16), '7/16"': (11.112, 25.4 / 14),
            '1/2"': (12.7, 25.4 / 12), '5/8"': (15.875, 25.4 / 11),
            '3/4"': (19.05, 25.4 / 10), '7/8"': (22.225, 25.4 / 9),
            '1"': (25.4, 25.4 / 8), '1 1/4"': (31.75, 25.4 / 8),
            '1 1/2"': (38.1, 25.4 / 8), '2"': (50.8, 25.4 / 7),
        },
    },
}

# ---------------- 螺纹公差带（精细程度） ----------------
# 外螺纹公差带：4g 最精密 → 10g 最松。6g 为最常用（与螺母 6H 配合）。
# 注：公差带主要影响牙顶/牙底极限偏差，建模以公称尺寸为准，仅作标注与信息展示。
THREAD_CLASSES = {
    "4g":  {"name": "4g (精密)", "desc": "精密级，公差最小，用于高精度场合"},
    "5g":  {"name": "5g (精密)", "desc": "精密级，公差较小"},
    "6e":  {"name": "6e (较松)", "desc": "较松公差，用于镀层较厚场合"},
    "6f":  {"name": "6f (中等偏松)", "desc": "中等偏松，用于中等镀层"},
    "6g":  {"name": "6g (中等·常用)", "desc": "中等精度，最常用（与螺母 6H 配合）"},
    "6h":  {"name": "6h (中等偏紧)", "desc": "中等精度，上偏差为零"},
    "8g":  {"name": "8g (较松)", "desc": "较松公差，用于长螺纹或镀层厚场合"},
    "10g": {"name": "10g (很松)", "desc": "很松公差，用于粗犷连接"},
}

# ---------------- 产品等级 ----------------
PRODUCT_GRADES = {
    "A": {"name": "A级 (高精度)", "desc": "螺纹直径 D≤M16 时采用，用于精密机器"},
    "B": {"name": "B级 (中等)", "desc": "螺纹直径 D>M16 时采用，用于一般机械"},
    "C": {"name": "C级 (低精度)", "desc": "用于表面粗糙、精度要求不高的结构"},
}

# ---------------- 性能等级 ----------------
PERFORMANCE_GRADES = {
    "4.6":  {"name": "4.6",  "desc": "抗拉 400MPa, 屈服 240MPa"},
    "4.8":  {"name": "4.8",  "desc": "抗拉 400MPa, 屈服 320MPa"},
    "5.6":  {"name": "5.6",  "desc": "抗拉 500MPa, 屈服 300MPa"},
    "5.8":  {"name": "5.8",  "desc": "抗拉 500MPa, 屈服 360MPa"},
    "6.8":  {"name": "6.8",  "desc": "抗拉 600MPa, 屈服 480MPa"},
    "8.8":  {"name": "8.8",  "desc": "抗拉 800MPa, 屈服 640MPa（最常用）"},
    "10.9": {"name": "10.9", "desc": "抗拉 1040MPa, 屈服 940MPa"},
    "12.9": {"name": "12.9", "desc": "抗拉 1220MPa, 屈服 1100MPa（最高）"},
}


def list_thread_systems():
    """返回 [(key, name, unit)]。"""
    return [(k, v["name"], v["unit"]) for k, v in THREAD_SYSTEMS.items()]


def thread_sizes(system_key):
    """返回某标准下的所有规格标签列表。"""
    sys_d = THREAD_SYSTEMS.get(system_key)
    if not sys_d:
        return []
    return list(sys_d["sizes"].keys())


def get_thread_spec(system_key, size_label):
    """返回 (d_mm, pitch_mm, angle_deg) 或 None。"""
    sys_d = THREAD_SYSTEMS.get(system_key)
    if not sys_d:
        return None
    t = sys_d["sizes"].get(size_label)
    if not t:
        return None
    return (t[0], t[1], sys_d["angle"])


def estimate_hex_dims(d):
    """由公称直径 d(mm) 估算六角对边 s 与头部高度 k（ISO 比例近似）。"""
    s = max(5.5, round(d * 1.5 / 0.5) * 0.5)
    k = round(d * 0.8 * 2) / 2.0
    return s, k


# 参数元信息：标签、单位、取值范围、类型、默认值、是否自动派生
PARAM_META = {
    "D":       {"label": "公称直径 D", "unit": "mm", "min": 1.0,  "default": 12.0, "type": "float"},
    "P":       {"label": "螺距 P",     "unit": "mm", "min": 0.1,  "default": 1.75, "type": "float"},
    "s":       {"label": "对边宽度 s", "unit": "mm", "min": 1.0,  "default": 18.0, "type": "float"},
    "e":       {"label": "对角宽度 e", "unit": "mm", "min": 1.0,  "default": 20.78, "type": "float", "auto": True,
                "note": "由 s 自动派生：六角 e=2s/√3，方螺母 e=s·√2，可覆盖"},
    "k":       {"label": "头部高度 k", "unit": "mm", "min": 0.5,  "default": 10.0, "type": "float"},
    "L":       {"label": "公称长度 L", "unit": "mm", "min": 5.0,  "default": 60.0, "type": "float"},
    "L1":      {"label": "螺纹长度 L₁", "unit": "mm", "min": 1.0, "default": 50.0, "type": "float",
                "note": "全螺纹时 = L - k；部分螺纹时由标准给出"},
    "dm1":     {"label": "头部外径 dm₁", "unit": "mm", "min": 1.0, "default": 18.0, "type": "float"},
    "dk":      {"label": "头径 dk", "unit": "mm", "min": 1.0, "default": 21.5, "type": "float"},
    "chamfer": {"label": "头部倒角 c", "unit": "mm", "min": 0.0,  "default": 0.8, "type": "float",
                "note": "0 = 不倒角"},
    "tip_chamfer": {"label": "末端倒角 cₜ", "unit": "mm", "min": 0.0, "default": 0.5, "type": "float",
                    "note": "螺纹末端 45° 锥形倒角，0 = 不倒角"},
    "thread":  {"label": "生成真实螺纹齿形", "unit": "", "default": True, "type": "bool"},
    "thread_angle": {"label": "螺纹牙型角", "unit": "°", "min": 30, "default": 60, "type": "float",
                    "note": "公制/UN 系列 60°；英制惠氏 BSW 55°"},
    "socket_depth": {"label": "内六角孔深", "unit": "mm", "min": 0.5, "default": 6.0, "type": "float"},
    "flange_d": {"label": "法兰外径 d_w", "unit": "mm", "min": 1.0, "default": 22.5, "type": "float"},
    "flange_t": {"label": "法兰厚度 t", "unit": "mm", "min": 0.5, "default": 2.5, "type": "float"},
    "head_angle": {"label": "沉头角度", "unit": "°", "min": 60, "default": 90.0, "type": "float",
                   "note": "沉头/半沉头锥面角度，标准为 90°"},
    "wing_span": {"label": "翼形翼展", "unit": "mm", "min": 10.0, "default": 40.0, "type": "float"},
    "ring_od": {"label": "吊环外径", "unit": "mm", "min": 8.0, "default": 30.0, "type": "float"},
    "ring_id": {"label": "吊环内径", "unit": "mm", "min": 4.0, "default": 18.0, "type": "float"},
    "neck_w":  {"label": "方颈宽", "unit": "mm", "min": 2.0, "default": 8.0, "type": "float"},
    "neck_h":  {"label": "方颈高", "unit": "mm", "min": 0.5, "default": 3.0, "type": "float"},
    "thread_class": {"label": "螺纹公差带", "unit": "", "default": "6g", "type": "choice",
                     "choices": list(THREAD_CLASSES.keys())},
    "product_grade": {"label": "产品等级", "unit": "", "default": "B", "type": "choice",
                      "choices": list(PRODUCT_GRADES.keys())},
    "performance": {"label": "性能等级", "unit": "", "default": "8.8", "type": "choice",
                    "choices": list(PERFORMANCE_GRADES.keys())},
    "fillet":  {"label": "倒圆(棱边圆角)", "unit": "", "default": False, "type": "bool"},
    "fillet_r": {"label": "圆角半径", "unit": "mm", "min": 0.1, "default": 0.5, "type": "float"},
}

# 每个螺栓类型：名称、标准、大类、外廓(shape)、说明、可见参数、默认值、标准尺寸预设
# 预设尺寸均含 P(螺距)、头部尺寸、默认 L 与 L1(螺纹长度)；单位 mm。
BOLT_TYPES = {
    "hex_full": {
        "key": "hex_full", "name": "六角头螺栓（全螺纹）", "en": "Hex bolt, full thread",
        "standard": "GB/T 5782 / ISO 4014 / DIN 933",
        "category": "hex", "shape": "hex_head",
        "desc": "六角头 + 杆部全长螺纹，最常用螺栓。",
        "params": ["D", "P", "s", "k", "L", "L1", "chamfer", "tip_chamfer", "thread"],
        "defaults": {"D": 12.0, "P": 1.75, "s": 18.0, "k": 10.0, "L": 60.0,
                     "L1": 50.0, "chamfer": 1.0, "tip_chamfer": 0.6, "thread": True},
        "presets": {
            "M5":  {"P": 0.8,  "s": 8.0,  "k": 4.7,  "L": 25.0,  "L1": 20.3},
            "M6":  {"P": 1.0,  "s": 10.0, "k": 5.2,  "L": 30.0,  "L1": 24.8},
            "M8":  {"P": 1.25, "s": 13.0, "k": 6.5,  "L": 40.0,  "L1": 33.5},
            "M10": {"P": 1.5,  "s": 16.0, "k": 8.0,  "L": 50.0,  "L1": 42.0},
            "M12": {"P": 1.75, "s": 18.0, "k": 10.0, "L": 60.0,  "L1": 50.0},
            "M14": {"P": 2.0,  "s": 21.0, "k": 11.5, "L": 70.0,  "L1": 58.5},
            "M16": {"P": 2.0,  "s": 24.0, "k": 13.0, "L": 80.0,  "L1": 67.0},
            "M20": {"P": 2.5,  "s": 30.0, "k": 16.0, "L": 90.0,  "L1": 74.0},
            "M24": {"P": 3.0,  "s": 36.0, "k": 19.0, "L": 100.0, "L1": 81.0},
            "M30": {"P": 3.5,  "s": 46.0, "k": 24.0, "L": 120.0, "L1": 96.0},
            "M36": {"P": 4.0,  "s": 55.0, "k": 29.0, "L": 130.0, "L1": 101.0},
        },
    },
    "hex_partial": {
        "key": "hex_partial", "name": "六角头螺栓（部分螺纹）", "en": "Hex bolt, partial thread",
        "standard": "GB/T 5783 / ISO 4017 / DIN 931",
        "category": "hex", "shape": "hex_head",
        "desc": "六角头 + 光杆 + 末端螺纹，用于有定位要求的连接。",
        "params": ["D", "P", "s", "k", "L", "L1", "chamfer", "tip_chamfer", "thread"],
        "defaults": {"D": 12.0, "P": 1.75, "s": 18.0, "k": 10.0, "L": 60.0,
                     "L1": 30.0, "chamfer": 1.0, "tip_chamfer": 0.6, "thread": True},
        "presets": {
            "M5":  {"P": 0.8,  "s": 8.0,  "k": 4.7,  "L": 40.0,  "L1": 23.0},
            "M6":  {"P": 1.0,  "s": 10.0, "k": 5.2,  "L": 45.0,  "L1": 25.0},
            "M8":  {"P": 1.25, "s": 13.0, "k": 6.5,  "L": 50.0,  "L1": 28.0},
            "M10": {"P": 1.5,  "s": 16.0, "k": 8.0,  "L": 60.0,  "L1": 33.0},
            "M12": {"P": 1.75, "s": 18.0, "k": 10.0, "L": 70.0,  "L1": 38.0},
            "M14": {"P": 2.0,  "s": 21.0, "k": 11.5, "L": 80.0,  "L1": 44.0},
            "M16": {"P": 2.0,  "s": 24.0, "k": 13.0, "L": 90.0,  "L1": 50.0},
            "M20": {"P": 2.5,  "s": 30.0, "k": 16.0, "L": 100.0, "L1": 56.0},
            "M24": {"P": 3.0,  "s": 36.0, "k": 19.0, "L": 110.0, "L1": 62.0},
            "M30": {"P": 3.5,  "s": 46.0, "k": 24.0, "L": 130.0, "L1": 72.0},
            "M36": {"P": 4.0,  "s": 55.0, "k": 29.0, "L": 140.0, "L1": 78.0},
        },
    },
    "hex_flange": {
        "key": "hex_flange", "name": "六角法兰面螺栓", "en": "Hex flange bolt",
        "standard": "GB/T 5787.2 / DIN 6921 / ISO 10667",
        "category": "hex", "shape": "hex_flange_head",
        "desc": "六角头 + 底部法兰盘，增大接触面积、防松性好。",
        "params": ["D", "P", "s", "k", "L", "L1", "flange_d", "flange_t", "chamfer", "tip_chamfer", "thread"],
        "defaults": {"D": 10.0, "P": 1.5, "s": 16.0, "k": 8.0, "L": 50.0, "L1": 42.0,
                     "flange_d": 17.5, "flange_t": 2.1, "chamfer": 0.8, "tip_chamfer": 0.5,
                     "thread": True},
        "presets": {
            "M5":  {"P": 0.8,  "s": 8.0,  "k": 4.7,  "L": 25.0,  "L1": 20.3, "flange_d": 9.7,  "flange_t": 1.3},
            "M6":  {"P": 1.0,  "s": 10.0, "k": 5.2,  "L": 30.0,  "L1": 24.8, "flange_d": 10.9, "flange_t": 1.5},
            "M8":  {"P": 1.25, "s": 13.0, "k": 6.5,  "L": 40.0,  "L1": 33.5, "flange_d": 14.4, "flange_t": 1.8},
            "M10": {"P": 1.5,  "s": 16.0, "k": 8.0,  "L": 50.0,  "L1": 42.0, "flange_d": 17.5, "flange_t": 2.1},
            "M12": {"P": 1.75, "s": 18.0, "k": 10.0, "L": 60.0,  "L1": 50.0, "flange_d": 20.5, "flange_t": 2.5},
            "M16": {"P": 2.0,  "s": 24.0, "k": 13.0, "L": 80.0,  "L1": 67.0, "flange_d": 26.5, "flange_t": 3.2},
            "M20": {"P": 2.5,  "s": 30.0, "k": 16.0, "L": 90.0,  "L1": 74.0, "flange_d": 33.0, "flange_t": 4.0},
        },
    },
    "socket_cap": {
        "key": "socket_cap", "name": "内六角圆柱头螺钉", "en": "Socket head cap screw",
        "standard": "GB/T 70.1 / ISO 4762 / DIN 912",
        "category": "socket", "shape": "cyl_head",
        "desc": "圆柱头 + 顶部内六角孔，高强度、占用空间小。",
        "params": ["D", "P", "dm1", "k", "s", "socket_depth", "L", "L1", "chamfer", "tip_chamfer", "thread"],
        "defaults": {"D": 10.0, "P": 1.5, "dm1": 15.0, "k": 10.0, "s": 10.0,
                     "socket_depth": 6.0, "L": 40.0, "L1": 30.0, "chamfer": 0.6,
                     "tip_chamfer": 0.5, "thread": True},
        "presets": {
            "M4":  {"P": 0.7,  "dm1": 6.0,  "k": 4.1,  "s": 4.0,  "socket_depth": 2.5, "L": 20.0, "L1": 15.9},
            "M5":  {"P": 0.8,  "dm1": 7.0,  "k": 5.0,  "s": 5.0,  "socket_depth": 3.0, "L": 25.0, "L1": 20.0},
            "M6":  {"P": 1.0,  "dm1": 9.0,  "k": 6.0,  "s": 5.0,  "socket_depth": 3.5, "L": 30.0, "L1": 24.0},
            "M8":  {"P": 1.25, "dm1": 12.0, "k": 8.0,  "s": 7.0,  "socket_depth": 5.0, "L": 40.0, "L1": 32.0},
            "M10": {"P": 1.5,  "dm1": 15.0, "k": 10.0, "s": 10.0, "socket_depth": 6.0, "L": 50.0, "L1": 40.0},
            "M12": {"P": 1.75, "dm1": 18.0, "k": 12.0, "s": 12.0, "socket_depth": 7.0, "L": 60.0, "L1": 48.0},
            "M16": {"P": 2.0,  "dm1": 24.0, "k": 16.0, "s": 16.0, "socket_depth": 9.0, "L": 80.0, "L1": 64.0},
            "M20": {"P": 2.5,  "dm1": 30.0, "k": 20.0, "s": 20.0, "socket_depth": 11.0, "L": 100.0, "L1": 80.0},
        },
    },
    "socket_pan": {
        "key": "socket_pan", "name": "内六角沉头螺钉", "en": "Socket countersunk screw",
        "standard": "GB/T 70.3 / ISO 4758 / DIN 915",
        "category": "socket", "shape": "cs_head",
        "desc": "90° 沉头锥面 + 顶部内六角孔，安装后头部与平面齐平。",
        "params": ["D", "P", "dk", "k", "s", "socket_depth", "head_angle", "L", "L1", "tip_chamfer", "thread"],
        "defaults": {"D": 8.0, "P": 1.25, "dk": 14.4, "k": 3.8, "s": 7.0,
                     "socket_depth": 2.5, "head_angle": 90.0, "L": 30.0, "L1": 26.2,
                     "tip_chamfer": 0.5, "thread": True},
        "presets": {
            "M4":  {"P": 0.7,  "dk": 7.7,  "k": 1.9,  "s": 4.0,  "socket_depth": 1.5, "L": 16.0, "L1": 14.1},
            "M5":  {"P": 0.8,  "dk": 9.1,  "k": 2.4,  "s": 5.0,  "socket_depth": 2.0, "L": 20.0, "L1": 17.6},
            "M6":  {"P": 1.0,  "dk": 10.9, "k": 3.0,  "s": 5.0,  "socket_depth": 2.5, "L": 25.0, "L1": 22.0},
            "M8":  {"P": 1.25, "dk": 14.4, "k": 3.8,  "s": 7.0,  "socket_depth": 3.0, "L": 30.0, "L1": 26.2},
            "M10": {"P": 1.5,  "dk": 18.0, "k": 4.6,  "s": 10.0, "socket_depth": 3.5, "L": 40.0, "L1": 35.4},
            "M12": {"P": 1.75, "dk": 21.5, "k": 5.4,  "s": 12.0, "socket_depth": 4.0, "L": 50.0, "L1": 44.6},
            "M16": {"P": 2.0,  "dk": 29.0, "k": 7.2,  "s": 16.0, "socket_depth": 5.5, "L": 60.0, "L1": 52.8},
        },
    },
    "socket_dome": {
        "key": "socket_dome", "name": "内六角杯头螺钉", "en": "Socket button head screw",
        "standard": "GB/T 70.4 / ISO 4759 / DIN 917",
        "category": "socket", "shape": "dome_head",
        "desc": "半球形杯头 + 顶部内六角孔，外观圆滑、防刮伤。",
        "params": ["D", "P", "dk", "k", "s", "socket_depth", "L", "L1", "tip_chamfer", "thread"],
        "defaults": {"D": 6.0, "P": 1.0, "dk": 10.0, "k": 2.0, "s": 4.0,
                     "socket_depth": 1.5, "L": 25.0, "L1": 23.0, "tip_chamfer": 0.5,
                     "thread": True},
        "presets": {
            "M4":  {"P": 0.7,  "dk": 6.7,  "k": 1.4,  "s": 3.0,  "socket_depth": 1.0, "L": 16.0, "L1": 14.6},
            "M5":  {"P": 0.8,  "dk": 8.3,  "k": 1.7,  "s": 4.0,  "socket_depth": 1.3, "L": 20.0, "L1": 18.3},
            "M6":  {"P": 1.0,  "dk": 10.0, "k": 2.0,  "s": 4.0,  "socket_depth": 1.5, "L": 25.0, "L1": 23.0},
            "M8":  {"P": 1.25, "dk": 13.5, "k": 2.5,  "s": 5.0,  "socket_depth": 2.0, "L": 30.0, "L1": 27.5},
            "M10": {"P": 1.5,  "dk": 16.7, "k": 3.1,  "s": 6.0,  "socket_depth": 2.5, "L": 40.0, "L1": 36.9},
            "M12": {"P": 1.75, "dk": 20.0, "k": 3.7,  "s": 8.0,  "socket_depth": 3.0, "L": 50.0, "L1": 46.3},
        },
    },
    "socket_hex": {
        "key": "socket_hex", "name": "内六角全六角螺钉", "en": "Hex socket full-hex screw",
        "standard": "GB/T 70.2-2008 近似",
        "category": "socket", "shape": "hex_socket_head",
        "desc": "外六角头 + 顶部内六角孔，便于两种工具驱动。",
        "params": ["D", "P", "s", "k", "socket_depth", "L", "L1", "chamfer", "tip_chamfer", "thread"],
        "defaults": {"D": 10.0, "P": 1.5, "s": 16.0, "k": 10.0, "socket_depth": 6.0,
                     "L": 50.0, "L1": 40.0, "chamfer": 0.6, "tip_chamfer": 0.5, "thread": True},
        "presets": {
            "M6":  {"P": 1.0,  "s": 10.0, "k": 6.0,  "socket_depth": 3.5, "L": 30.0, "L1": 24.0},
            "M8":  {"P": 1.25, "s": 13.0, "k": 8.0,  "socket_depth": 5.0, "L": 40.0, "L1": 32.0},
            "M10": {"P": 1.5,  "s": 16.0, "k": 10.0, "socket_depth": 6.0, "L": 50.0, "L1": 40.0},
            "M12": {"P": 1.75, "s": 18.0, "k": 12.0, "socket_depth": 7.0, "L": 60.0, "L1": 48.0},
            "M16": {"P": 2.0,  "s": 24.0, "k": 16.0, "socket_depth": 9.0, "L": 80.0, "L1": 64.0},
            "M20": {"P": 2.5,  "s": 30.0, "k": 20.0, "socket_depth": 11.0, "L": 100.0, "L1": 80.0},
        },
    },
    "square_head": {
        "key": "square_head", "name": "方头螺栓", "en": "Square head bolt",
        "standard": "GB/T 5787 / 近似 ISO 4028",
        "category": "hex", "shape": "square_head",
        "desc": "方形头，用于槽钢、木结构等特殊场合。",
        "params": ["D", "P", "s", "k", "L", "L1", "chamfer", "tip_chamfer", "thread"],
        "defaults": {"D": 12.0, "P": 1.75, "s": 15.5, "k": 6.4, "L": 50.0,
                     "L1": 43.6, "chamfer": 0.6, "tip_chamfer": 0.5, "thread": True},
        "presets": {
            "M5":  {"P": 0.8,  "s": 7.0,  "k": 3.0,  "L": 25.0, "L1": 22.0},
            "M6":  {"P": 1.0,  "s": 8.5,  "k": 3.6,  "L": 30.0, "L1": 26.4},
            "M8":  {"P": 1.25, "s": 11.0, "k": 4.5,  "L": 40.0, "L1": 35.5},
            "M10": {"P": 1.5,  "s": 13.5, "k": 5.6,  "L": 50.0, "L1": 44.4},
            "M12": {"P": 1.75, "s": 15.5, "k": 6.4,  "L": 60.0, "L1": 53.6},
            "M16": {"P": 2.0,  "s": 21.0, "k": 8.5,  "L": 80.0, "L1": 71.5},
            "M20": {"P": 2.5,  "s": 25.0, "k": 10.5, "L": 100.0, "L1": 89.5},
        },
    },
    "button_head": {
        "key": "button_head", "name": "圆柱头螺栓", "en": "Button head bolt",
        "standard": "GB/T 818 / 近似 ISO 4023 / DIN 404",
        "category": "round", "shape": "button_head",
        "desc": "圆柱头（顶部圆滑），外观美观，常用于薄板连接。",
        "params": ["D", "P", "dk", "k", "L", "L1", "tip_chamfer", "thread"],
        "defaults": {"D": 6.0, "P": 1.0, "dk": 10.0, "k": 2.0, "L": 25.0,
                     "L1": 23.0, "tip_chamfer": 0.5, "thread": True},
        "presets": {
            "M3":  {"P": 0.5,  "dk": 5.1,  "k": 1.1,  "L": 12.0, "L1": 10.9},
            "M4":  {"P": 0.7,  "dk": 6.7,  "k": 1.4,  "L": 16.0, "L1": 14.6},
            "M5":  {"P": 0.8,  "dk": 8.3,  "k": 1.7,  "L": 20.0, "L1": 18.3},
            "M6":  {"P": 1.0,  "dk": 10.0, "k": 2.0,  "L": 25.0, "L1": 23.0},
            "M8":  {"P": 1.25, "dk": 13.5, "k": 2.5,  "L": 30.0, "L1": 27.5},
            "M10": {"P": 1.5,  "dk": 16.7, "k": 3.1,  "L": 40.0, "L1": 36.9},
            "M12": {"P": 1.75, "dk": 20.0, "k": 3.7,  "L": 50.0, "L1": 46.3},
            "M16": {"P": 2.0,  "dk": 26.5, "k": 4.8,  "L": 60.0, "L1": 55.2},
        },
    },
    "countersunk": {
        "key": "countersunk", "name": "沉头螺栓", "en": "Countersunk bolt",
        "standard": "GB/T 5780 / 近似 ISO 4030 / DIN 617",
        "category": "round", "shape": "cs_head",
        "desc": "90° 沉头锥面，安装后头部与平面齐平，无突出。",
        "params": ["D", "P", "dk", "k", "head_angle", "L", "L1", "tip_chamfer", "thread"],
        "defaults": {"D": 8.0, "P": 1.25, "dk": 14.4, "k": 3.8, "head_angle": 90.0,
                     "L": 30.0, "L1": 26.2, "tip_chamfer": 0.5, "thread": True},
        "presets": {
            "M4":  {"P": 0.7,  "dk": 7.7,  "k": 1.9,  "L": 16.0, "L1": 14.1},
            "M5":  {"P": 0.8,  "dk": 9.1,  "k": 2.4,  "L": 20.0, "L1": 17.6},
            "M6":  {"P": 1.0,  "dk": 10.9, "k": 3.0,  "L": 25.0, "L1": 22.0},
            "M8":  {"P": 1.25, "dk": 14.4, "k": 3.8,  "L": 30.0, "L1": 26.2},
            "M10": {"P": 1.5,  "dk": 18.0, "k": 4.6,  "L": 40.0, "L1": 35.4},
            "M12": {"P": 1.75, "dk": 21.5, "k": 5.4,  "L": 50.0, "L1": 44.6},
            "M16": {"P": 2.0,  "dk": 29.0, "k": 7.2,  "L": 60.0, "L1": 52.8},
            "M20": {"P": 2.5,  "dk": 36.0, "k": 9.0,  "L": 70.0, "L1": 61.0},
            "M24": {"P": 3.0,  "dk": 43.0, "k": 10.5, "L": 80.0, "L1": 69.5},
        },
    },
    "pan_head": {
        "key": "pan_head", "name": "半沉头螺栓", "en": "Pan head bolt",
        "standard": "GB/T 5785 / 近似",
        "category": "round", "shape": "pan_head",
        "desc": "盘头 + 锥面过渡，头部略高于平面，兼顾强度与美观。",
        "params": ["D", "P", "dk", "k", "head_angle", "L", "L1", "tip_chamfer", "thread"],
        "defaults": {"D": 6.0, "P": 1.0, "dk": 11.0, "k": 4.2, "head_angle": 90.0,
                     "L": 25.0, "L1": 20.8, "tip_chamfer": 0.5, "thread": True},
        "presets": {
            "M3":  {"P": 0.5,  "dk": 6.0,  "k": 2.0,  "L": 12.0, "L1": 10.0},
            "M4":  {"P": 0.7,  "dk": 7.8,  "k": 2.8,  "L": 16.0, "L1": 13.2},
            "M5":  {"P": 0.8,  "dk": 9.5,  "k": 3.4,  "L": 20.0, "L1": 16.6},
            "M6":  {"P": 1.0,  "dk": 11.0, "k": 4.2,  "L": 25.0, "L1": 20.8},
            "M8":  {"P": 1.25, "dk": 14.5, "k": 5.2,  "L": 30.0, "L1": 24.8},
            "M10": {"P": 1.5,  "dk": 18.0, "k": 6.5,  "L": 40.0, "L1": 33.5},
            "M12": {"P": 1.75, "dk": 22.0, "k": 8.0,  "L": 50.0, "L1": 42.0},
        },
    },
    "carriage": {
        "key": "carriage", "name": "马车螺栓", "en": "Carriage bolt",
        "standard": "GB/T 5781 / 近似",
        "category": "round", "shape": "carriage_head",
        "desc": "圆头 + 下方方颈，防旋转，用于木材/薄板连接。",
        "params": ["D", "P", "dk", "k", "neck_w", "neck_h", "L", "L1", "tip_chamfer", "thread"],
        "defaults": {"D": 8.0, "P": 1.25, "dk": 14.0, "k": 5.0, "neck_w": 7.5,
                     "neck_h": 3.5, "L": 40.0, "L1": 25.0, "tip_chamfer": 0.5, "thread": True},
        "presets": {
            "M5":  {"P": 0.8,  "dk": 9.0,  "k": 3.0,  "neck_w": 4.8, "neck_h": 2.5, "L": 25.0, "L1": 16.0},
            "M6":  {"P": 1.0,  "dk": 10.5, "k": 3.8,  "neck_w": 5.8, "neck_h": 3.0, "L": 30.0, "L1": 18.0},
            "M8":  {"P": 1.25, "dk": 14.0, "k": 5.0,  "neck_w": 7.5, "neck_h": 3.5, "L": 40.0, "L1": 25.0},
            "M10": {"P": 1.5,  "dk": 17.0, "k": 6.0,  "neck_w": 9.0, "neck_h": 4.0, "L": 50.0, "L1": 30.0},
            "M12": {"P": 1.75, "dk": 20.0, "k": 7.0,  "neck_w": 11.0,"neck_h": 4.5, "L": 60.0, "L1": 35.0},
            "M16": {"P": 2.0,  "dk": 26.0, "k": 9.0,  "neck_w": 14.0,"neck_h": 5.5, "L": 70.0, "L1": 42.0},
            "M20": {"P": 2.5,  "dk": 32.0, "k": 11.0, "neck_w": 18.0,"neck_h": 6.5, "L": 80.0, "L1": 48.0},
        },
    },
    "eye_bolt": {
        "key": "eye_bolt", "name": "吊环螺栓", "en": "Eye bolt",
        "standard": "GB/T 823 / 近似",
        "category": "special", "shape": "eye_head",
        "desc": "顶部吊环 + 螺杆，用于吊装/牵引。",
        "params": ["D", "P", "ring_od", "ring_id", "k", "L", "L1", "tip_chamfer", "thread"],
        "defaults": {"D": 10.0, "P": 1.5, "ring_od": 20.0, "ring_id": 10.0, "k": 6.0,
                     "L": 40.0, "L1": 30.0, "tip_chamfer": 0.5, "thread": True},
        "presets": {
            "M6":  {"P": 1.0,  "ring_od": 12.5, "ring_id": 6.0,  "k": 5.0,  "L": 25.0, "L1": 20.0},
            "M8":  {"P": 1.25, "ring_od": 16.0, "ring_id": 8.0,  "k": 6.0,  "L": 30.0, "L1": 24.0},
            "M10": {"P": 1.5,  "ring_od": 20.0, "ring_id": 10.0, "k": 7.0,  "L": 40.0, "L1": 30.0},
            "M12": {"P": 1.75, "ring_od": 24.0, "ring_id": 12.0, "k": 8.0,  "L": 50.0, "L1": 38.0},
            "M16": {"P": 2.0,  "ring_od": 32.0, "ring_id": 16.0, "k": 10.0, "L": 60.0, "L1": 44.0},
            "M20": {"P": 2.5,  "ring_od": 40.0, "ring_id": 20.0, "k": 12.0, "L": 70.0, "L1": 54.0},
            "M24": {"P": 3.0,  "ring_od": 48.0, "ring_id": 24.0, "k": 14.0, "L": 80.0, "L1": 61.0},
        },
    },
    "stud": {
        "key": "stud", "name": "双头螺柱", "en": "Stud bolt",
        "standard": "GB/T 896 / ISO 899 / DIN 976",
        "category": "special", "shape": "stud",
        "desc": "两端螺纹 + 中间光杆，用于盲孔/贯穿连接，可两端上螺母。",
        "params": ["D", "P", "L", "b", "tip_chamfer", "thread"],
        "defaults": {"D": 12.0, "P": 1.75, "L": 80.0, "b": 26.0,
                     "tip_chamfer": 0.6, "thread": True},
        "presets": {
            "M6":  {"P": 1.0,  "L": 50.0, "b": 14.0},
            "M8":  {"P": 1.25, "L": 60.0, "b": 18.0},
            "M10": {"P": 1.5,  "L": 70.0, "b": 22.0},
            "M12": {"P": 1.75, "L": 80.0, "b": 26.0},
            "M16": {"P": 2.0,  "L": 100.0, "b": 34.0},
            "M20": {"P": 2.5,  "L": 120.0, "b": 44.0},
            "M24": {"P": 3.0,  "L": 140.0, "b": 52.0},
            "M30": {"P": 3.5,  "L": 160.0, "b": 64.0},
            "M36": {"P": 4.0,  "L": 180.0, "b": 76.0},
            "M42": {"P": 4.5,  "L": 200.0, "b": 88.0},
            "M48": {"P": 5.0,  "L": 220.0, "b": 100.0},
        },
    },
    "wing_bolt": {
        "key": "wing_bolt", "name": "翼形螺栓", "en": "Wing bolt / butterfly bolt",
        "standard": "GB/T 25159 / 近似",
        "category": "special", "shape": "wing_head",
        "desc": "两侧蝶翼便于手拧，无需工具，用于快速拆装。",
        "params": ["D", "P", "wing_span", "k", "L", "L1", "tip_chamfer", "thread"],
        "defaults": {"D": 8.0, "P": 1.25, "wing_span": 36.0, "k": 5.0,
                     "L": 40.0, "L1": 35.0, "tip_chamfer": 0.5, "thread": True},
        "presets": {
            "M4":  {"P": 0.7,  "wing_span": 22.0, "k": 3.0,  "L": 20.0, "L1": 17.0},
            "M5":  {"P": 0.8,  "wing_span": 26.0, "k": 3.5,  "L": 25.0, "L1": 21.5},
            "M6":  {"P": 1.0,  "wing_span": 30.0, "k": 4.0,  "L": 30.0, "L1": 26.0},
            "M8":  {"P": 1.25, "wing_span": 36.0, "k": 5.0,  "L": 40.0, "L1": 35.0},
            "M10": {"P": 1.5,  "wing_span": 42.0, "k": 6.0,  "L": 50.0, "L1": 44.0},
            "M12": {"P": 1.75, "wing_span": 48.0, "k": 7.0,  "L": 60.0, "L1": 53.0},
        },
    },
}


def derive_e(s, shape="hex_head"):
    """由对边宽度 s 派生对角宽度 e。六角 e = 2s/√3；方头 e = s·√2。"""
    if shape == "square_head":
        return s * math.sqrt(2.0)
    return 2.0 * s / math.sqrt(3.0)


def _parse_D_from_label(label):
    """从规格标签解析公称直径 D，例如 'M20×1.5' -> 20.0，'1/2"-13' -> 12.7。"""
    s = str(label)
    # 英制带分数: 1 1/4" -> 31.75（先于普通分数匹配）
    mf = re.match(r"(\d+)\s+(\d+)\s*/\s*(\d+)", s)
    if mf:
        whole = float(mf.group(1))
        frac = float(mf.group(2)) / float(mf.group(3))
        return (whole + frac) * 25.4
    # 英制分数规格: 1/2" -> 12.7（优先于公制正则，避免误取分子）
    fm = re.match(r"(\d+)\s*/\s*(\d+)", s)
    if fm:
        return float(fm.group(1)) / float(fm.group(2)) * 25.4
    m = re.search(r"M?\s*(\d+(?:\.\d+)?)", s)
    if m:
        return float(m.group(1))
    return None


def get_type(key):
    return BOLT_TYPES.get(key)


def default_params(key):
    """返回某类型带 e、bore 派生值后的完整参数 dict。"""
    t = BOLT_TYPES[key]
    p = dict(t["defaults"])
    if "s" in p:
        p["e"] = derive_e(p["s"], t.get("shape", "hex_head"))
    return p


def apply_preset(key, size_label):
    """套用标准尺寸预设，返回合并后的参数字典（含派生的 e）。"""
    t = BOLT_TYPES[key]
    p = dict(t["defaults"])
    if size_label in t.get("presets", {}):
        p.update(t["presets"][size_label])
        d = _parse_D_from_label(size_label)
        if d is not None:
            p["D"] = d
    # 全螺纹类型默认 L1 = L - k
    if p.get("L1", 0) <= 0 and "k" in p:
        p["L1"] = max(1.0, p["L"] - p["k"])
    if "s" in p:
        p["e"] = derive_e(p["s"], t.get("shape", "hex_head"))
    return p


def visible_params(key):
    """返回该类型在 UI 中展示的参数键列表。"""
    return BOLT_TYPES[key]["params"]
