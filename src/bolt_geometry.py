# -*- coding: utf-8 -*-
"""
纯几何计算模块（与 CATIA 解耦，可离线运行/测试）。

职责：根据螺栓类型 + 参数，计算 2D 轮廓并生成一组"建模步骤(steps)"。
CATIA 控制器 (catia_controller.py) 只负责把这些步骤翻译成 CATIA COM 调用；
本模块本身不依赖 pywin32 / CATIA。

================= 建模坐标系约定（重要） =================
CATIA late-binding 下**取不到实体的面引用**（.Faces / Selection.Search 均
不可用），因此所有特征必须能在固定的原始平面（XY/ZX）上完成。为此统一约定：

    z = 0     -> 螺栓"头部顶面"（草图基准面 = PlaneXY）
    z = -k    -> 头部底面 / 螺纹起点（全螺纹时）
    z = -L    -> 螺栓末端（螺纹收尾 + 末端倒角）

外螺纹（螺栓）与内螺纹（螺母）的关键区别：
    螺栓 = 圆柱杆(直径 D=大径) + 在杆上切 V 形牙槽(切到小径 d1)。
    牙型三角形顶点(牙槽最深处)在小径侧 minor_r，底边在大径侧 major_r；
    与螺母(顶点在 major_r)正好相反。

轮廓表示：
  polygon : {"kind":"polygon", "points":[(x,y), ...]}            闭合多边形
  circle  : {"kind":"circle",  "cx":0.0, "cy":0.0, "r":r}        闭合圆

建模步骤 op：
  "pad"      : 从 z=0 向 -Z 拉伸 length（头部/杆部主体）。outer 必填。
  "pad_up"   : 从 z=0 向 +Z 拉伸 length（吊环等顶部特征）。
  "pocket"   : 从 z=0 向 -Z 挖槽 depth（内六角孔等顶面凹槽）。
  "thread"   : 真实外螺纹齿形（牙型 60°/55°），由"多牙型轮廓一次 Groove
               旋转切除"实现（major_r=大径/2, minor_r=小径/2, pitch,
               depth, angle, z_start=螺纹起点）。
  "chamfer"  : 45° 锥形倒角（size），由 Groove 旋转切除实现，需 outer_r；
               faces=("top",) 头部顶面倒角，("bottom",) 末端倒角。
  "fillet"   : 棱边倒圆（radius）。
"""

import math

from bolt_data import BOLT_TYPES, derive_e


# ---------- 基础轮廓生成 ----------

def _hex_points(s, rot_deg=0.0):
    """正多边形(6边)顶点，flat-to-flat = s，顶点在角度 rot 起算。"""
    R = s / math.sqrt(3.0)  # 外接圆半径（中心->顶点）
    return [(R * math.cos(math.radians(60.0 * i + rot_deg)),
             R * math.sin(math.radians(60.0 * i + rot_deg))) for i in range(6)]


def _square_points(s, rot_deg=45.0):
    """正方形顶点，flat-to-flat = s。默认旋转 45° 使边与坐标轴平行。"""
    r = s / math.sqrt(2.0)
    return [(r * math.cos(math.radians(rot_deg + 90.0 * i)),
             r * math.sin(math.radians(rot_deg + 90.0 * i))) for i in range(4)]


def _circle(cx, cy, r):
    return {"kind": "circle", "cx": cx, "cy": cy, "r": r}


def _polygon(points):
    return {"kind": "polygon", "points": [tuple(p) for p in points]}


def _rect_at(w, l, cx, cy):
    """以 (cx,cy) 为中心的矩形（翼形螺栓蝶翼）。"""
    hw, hl = w / 2.0, l / 2.0
    return _polygon([(cx - hw, cy - hl), (cx + hw, cy - hl),
                     (cx + hw, cy + hl), (cx - hw, cy + hl)])


def _thread_height(P, angle=60.0):
    """ISO 基本牙型牙高 h = 5H/8，H = P / (2·tan(a/2))。

    60° 公制/UN : h ≈ 0.54127·P
    55° 英制 BSW: h ≈ 0.64033·P
    牙型的"半底宽"恒为 5P/16 = 0.3125·P（与牙型角无关）。
    """
    a = math.radians(float(angle))
    if a <= 0:
        return 0.54127 * P
    H = P / (2.0 * math.tan(a / 2.0))
    return 5.0 * H / 8.0


def _outer_radius(contour):
    """轮廓的最大外接半径（倒角锥面用）。"""
    if contour["kind"] == "circle":
        return abs(contour["cx"]) + contour["r"]
    return max(math.hypot(x, y) for (x, y) in contour["points"])


# ---------- 主建模函数 ----------

def build_model(type_key, params):
    """返回建模模型 dict：{name, type_key, steps, preview}。"""
    t = BOLT_TYPES[type_key]
    D = float(params.get("D", 12.0))
    P = float(params.get("P", 1.75))
    L = float(params.get("L", 60.0))
    k = float(params.get("k", 10.0))
    L1 = float(params.get("L1", max(1.0, L - k)))
    chamfer = float(params.get("chamfer", 0.8))
    tip_chamfer = float(params.get("tip_chamfer", 0.5))
    thread = bool(params.get("thread", False))
    thread_angle = float(params.get("thread_angle", 60.0))
    fillet = bool(params.get("fillet", False))
    fillet_r = float(params.get("fillet_r", 0.5))

    steps = []
    th_h = _thread_height(P, thread_angle)   # 牙高
    shank_r = D / 2.0
    minor_r = max(shank_r - th_h, shank_r * 0.4)   # 外螺纹小径(牙槽底)

    def add_thread(z_start, depth):
        """外螺纹齿形：在杆上从 z_start 向下切 depth 长。"""
        if not thread or depth <= 0.5:
            return
        steps.append({"op": "thread",
                      "major_r": shank_r, "minor_r": minor_r,
                      "pitch": P, "depth": depth, "angle": thread_angle,
                      "z_start": float(z_start)})

    def add_head_chamfer(R):
        if chamfer and chamfer > 0 and R > chamfer * 1.2:
            steps.append({"op": "chamfer", "size": chamfer,
                          "outer_r": R, "top_z": 0.0, "bottom_z": -L,
                          "faces": ["top"]})

    def add_tip_chamfer(R=None):
        r = R or shank_r
        if tip_chamfer and tip_chamfer > 0 and r > tip_chamfer * 1.2:
            steps.append({"op": "chamfer", "size": tip_chamfer,
                          "outer_r": r, "top_z": 0.0, "bottom_z": -L,
                          "faces": ["bottom"]})

    def add_fillet_if():
        if fillet and fillet_r and fillet_r > 0:
            r = fillet_r
            if thread:
                r = min(r, max(0.05, th_h * 0.25))
            steps.append({"op": "fillet", "radius": r})

    def add_socket_if(s, depth):
        """内六角孔：顶面正六边形凹槽。"""
        sd = float(params.get("socket_depth", depth))
        sd = min(sd, max(0.5, k * 0.8))
        steps.append({"op": "pocket", "face": "top",
                      "contour": _polygon(_hex_points(s)), "depth": sd})

    # 杆部圆柱（所有带杆类型共用；从 z=0 贯穿到 z=-L，与头部布尔并集）
    def add_shank(r=None):
        steps.append({"op": "pad", "plane": "XY",
                      "outer": _circle(0, 0, r or shank_r),
                      "inner": None, "length": L})

    # ---------------- 六角头 / 方头（全螺纹 / 部分螺纹） ----------------
    if type_key in ("hex_full", "hex_partial", "square_head"):
        s = float(params.get("s", 18.0))
        outer = _polygon(_square_points(s)) if type_key == "square_head" \
            else _polygon(_hex_points(s))
        R = _outer_radius(outer)
        steps.append({"op": "pad", "plane": "XY", "outer": outer,
                      "inner": None, "length": k})
        add_shank()
        full = (type_key == "hex_full")
        z_start = -k if full else -(L - L1)
        add_thread(z_start, (L - k) if full else L1)
        add_head_chamfer(R)
        add_tip_chamfer()
        add_fillet_if()

    # ---------------- 六角法兰面螺栓 ----------------
    elif type_key == "hex_flange":
        s = float(params.get("s", 16.0))
        flange_d = float(params.get("flange_d", derive_e(s) * 1.3))
        flange_t = float(params.get("flange_t", 2.5))
        hexo = _polygon(_hex_points(s))
        R = _outer_radius(hexo)
        # 底部法兰盘拉满头部高度；六角体短 flange_t，露出底部法兰
        steps.append({"op": "pad", "plane": "XY",
                      "outer": _circle(0, 0, flange_d / 2.0),
                      "inner": None, "length": k})
        steps.append({"op": "pad", "plane": "XY", "outer": hexo,
                      "inner": None, "length": max(0.5, k - flange_t)})
        add_shank()
        add_thread(-k, L - k)
        add_head_chamfer(R)
        add_tip_chamfer()
        add_fillet_if()

    # ---------------- 内六角圆柱头 ----------------
    elif type_key == "socket_cap":
        dm1 = float(params.get("dm1", 15.0))
        s = float(params.get("s", 10.0))
        steps.append({"op": "pad", "plane": "XY",
                      "outer": _circle(0, 0, dm1 / 2.0),
                      "inner": None, "length": k})
        add_shank()
        add_socket_if(s, params.get("socket_depth", 6.0))
        add_thread(-k, L - k)
        add_head_chamfer(dm1 / 2.0)
        add_tip_chamfer()
        add_fillet_if()

    # ---------------- 内六角沉头（锥面） ----------------
    elif type_key == "socket_pan":
        dk = float(params.get("dk", 14.4))
        s = float(params.get("s", 7.0))
        steps.append({"op": "pad", "plane": "XY",
                      "outer": _circle(0, 0, dk / 2.0),
                      "inner": None, "length": k})
        add_shank()
        add_socket_if(s, params.get("socket_depth", 3.0))
        # 90° 沉头锥面：顶部 45° 倒角逼近锥形
        c = min(k * 0.95, dk / 2.0 * 0.9)
        if c > 0.5:
            steps.append({"op": "chamfer", "size": c,
                          "outer_r": dk / 2.0, "top_z": 0.0, "bottom_z": -L,
                          "faces": ["top"]})
        add_thread(-k, L - k)
        add_tip_chamfer()
        add_fillet_if()

    # ---------------- 内六角杯头（圆顶） ----------------
    elif type_key == "socket_dome":
        dk = float(params.get("dk", 10.0))
        s = float(params.get("s", 4.0))
        steps.append({"op": "pad", "plane": "XY",
                      "outer": _circle(0, 0, dk / 2.0),
                      "inner": None, "length": k})
        add_shank()
        add_socket_if(s, params.get("socket_depth", 2.0))
        # 圆顶：大倒角逼近
        c = min(k * 0.85, dk / 2.0 * 0.8)
        if c > 0.5:
            steps.append({"op": "chamfer", "size": c,
                          "outer_r": dk / 2.0, "top_z": 0.0, "bottom_z": -L,
                          "faces": ["top"]})
        add_thread(-k, L - k)
        add_tip_chamfer()
        add_fillet_if()

    # ---------------- 内六角全六角（外六角 + 内六角孔） ----------------
    elif type_key == "socket_hex":
        s = float(params.get("s", 16.0))
        outer = _polygon(_hex_points(s))
        R = _outer_radius(outer)
        steps.append({"op": "pad", "plane": "XY", "outer": outer,
                      "inner": None, "length": k})
        add_shank()
        add_socket_if(s * 0.72, params.get("socket_depth", 6.0))
        add_thread(-k, L - k)
        add_head_chamfer(R)
        add_tip_chamfer()
        add_fillet_if()

    # ---------------- 圆柱头（圆头/按钮头） ----------------
    elif type_key == "button_head":
        dk = float(params.get("dk", 10.0))
        steps.append({"op": "pad", "plane": "XY",
                      "outer": _circle(0, 0, dk / 2.0),
                      "inner": None, "length": k})
        add_shank()
        # 顶部圆滑：45° 倒角逼近
        c = min(k * 0.6, (dk - D) / 2.0 * 0.9)
        if c > 0.5:
            steps.append({"op": "chamfer", "size": c,
                          "outer_r": dk / 2.0, "top_z": 0.0, "bottom_z": -L,
                          "faces": ["top"]})
        add_thread(-k, L - k)
        add_tip_chamfer()
        add_fillet_if()

    # ---------------- 沉头螺栓（90° 锥面） ----------------
    elif type_key == "countersunk":
        dk = float(params.get("dk", 14.4))
        head_angle = float(params.get("head_angle", 90.0))
        steps.append({"op": "pad", "plane": "XY",
                      "outer": _circle(0, 0, dk / 2.0),
                      "inner": None, "length": k})
        add_shank()
        # 90° 沉头：顶部 45° 倒角(≈锥面半角)，倒角量 = k
        c = min(k * 0.98, dk / 2.0 * 0.95)
        if c > 0.5:
            steps.append({"op": "chamfer", "size": c,
                          "outer_r": dk / 2.0, "top_z": 0.0, "bottom_z": -L,
                          "faces": ["top"]})
        z_start = -(L - L1)
        add_thread(z_start, L1)
        add_tip_chamfer()
        add_fillet_if()

    # ---------------- 半沉头螺栓（盘头 + 锥面过渡） ----------------
    elif type_key == "pan_head":
        dk = float(params.get("dk", 11.0))
        head_angle = float(params.get("head_angle", 90.0))
        steps.append({"op": "pad", "plane": "XY",
                      "outer": _circle(0, 0, dk / 2.0),
                      "inner": None, "length": k})
        add_shank()
        c = min(k * 0.7, (dk - D) / 2.0)
        if c > 0.5:
            steps.append({"op": "chamfer", "size": c,
                          "outer_r": dk / 2.0, "top_z": 0.0, "bottom_z": -L,
                          "faces": ["top"]})
        z_start = -(L - L1)
        add_thread(z_start, L1)
        add_tip_chamfer()
        add_fillet_if()

    # ---------------- 马车螺栓（圆头 + 方颈） ----------------
    elif type_key == "carriage":
        dk = float(params.get("dk", 14.0))
        neck_w = float(params.get("neck_w", 7.5))
        neck_h = float(params.get("neck_h", 3.5))
        steps.append({"op": "pad", "plane": "XY",
                      "outer": _circle(0, 0, dk / 2.0),
                      "inner": None, "length": k})
        # 方颈：从头部向下延伸 neck_h（用贯穿 pad + 并集）
        steps.append({"op": "pad", "plane": "XY",
                      "outer": _polygon(_square_points(neck_w)),
                      "inner": None, "length": k + neck_h})
        add_shank()
        z_start = -(L - L1)
        add_thread(z_start, L1)
        add_tip_chamfer()
        add_fillet_if()

    # ---------------- 吊环螺栓 ----------------
    elif type_key == "eye_bolt":
        ring_od = float(params.get("ring_od", 20.0))
        ring_id = float(params.get("ring_id", 10.0))
        steps.append({"op": "pad_up", "plane": "XY",
                      "outer": _circle(0, 0, ring_od / 2.0),
                      "inner": _circle(0, 0, ring_id / 2.0),
                      "length": k})
        # 连接杆（直径取 D 与环内径 0.85 的较大者，保持吊环中孔通透）
        boss_r = max(shank_r, ring_id * 0.45)
        add_shank(boss_r)
        z_start = -(L - L1)
        add_thread(z_start, L1)
        add_tip_chamfer(boss_r)
        add_fillet_if()

    # ---------------- 双头螺柱（两端螺纹 + 中间光杆） ----------------
    elif type_key == "stud":
        b = float(params.get("b", 26.0))
        b = min(b, L / 2.0 - 1.0)
        add_shank()
        add_thread(0.0, b)              # 上端螺纹
        add_thread(-(L - b), b)         # 下端螺纹
        add_tip_chamfer()
        add_fillet_if()

    # ---------------- 翼形螺栓 ----------------
    elif type_key == "wing_bolt":
        wing_span = float(params.get("wing_span", 36.0))
        boss_r = max(shank_r * 1.2, 4.0)
        steps.append({"op": "pad", "plane": "XY",
                      "outer": _circle(0, 0, boss_r), "inner": None, "length": k})
        wing_w = wing_span * 0.26
        wing_h = wing_span * 0.30
        cx_w = wing_span / 2.0 - wing_w / 2.0
        for sign in (1.0, -1.0):
            steps.append({"op": "pad", "plane": "XY",
                          "outer": _rect_at(wing_w, wing_h, sign * cx_w, 0.0),
                          "inner": None, "length": k})
        add_shank()
        z_start = -(L - L1)
        add_thread(z_start, L1)
        add_tip_chamfer()
        add_fillet_if()

    else:
        raise ValueError(f"未知螺栓类型: {type_key}")

    # 预览信息：取头部轮廓（顶视）+ 杆径
    first = next((st for st in steps if st["op"] in ("pad", "pad_up")), None)
    preview = {
        "outer": first["outer"] if first else _circle(0, 0, shank_r),
        "shank_r": shank_r,
        "type_key": type_key,
        "L": L, "k": k,
    }

    size_label = f"M{format_num(D)}"
    name = f"{t['name']}_{size_label}x{format_num(L)}"
    return {"name": name, "type_key": type_key, "steps": steps, "preview": preview}


def format_num(v):
    """把浮点格式化为去掉多余小数的字符串，如 12.0 -> '12'。"""
    if abs(v - round(v)) < 1e-6:
        return str(int(round(v)))
    return f"{v:g}"


def preview_outline(type_key, params):
    """便捷函数：仅返回预览轮廓（外轮廓 + 杆径半径）。"""
    return build_model(type_key, params)["preview"]


def list_all_types():
    """返回 (category, [type_keys]) 分组列表，供 UI 树展示。"""
    from bolt_data import BOLT_CATEGORIES
    out = []
    for cat_key, cat_name in BOLT_CATEGORIES:
        keys = [k for k, v in BOLT_TYPES.items() if v["category"] == cat_key]
        if keys:
            out.append((cat_key, cat_name, keys))
    return out
