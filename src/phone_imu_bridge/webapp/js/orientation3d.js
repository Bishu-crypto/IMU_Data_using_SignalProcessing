/**
 * orientation3d.js — 3D wireframe cube rendered via Canvas 2D.
 * Rotated by quaternion data from the Madgwick filter to show phone orientation.
 */

const Orientation3D = (() => {
    let canvas, ctx;
    let quat = { w: 1, x: 0, y: 0, z: 0 };

    // Unit cube vertices centered at origin
    const VERTS = [
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1,  1], [1, -1,  1], [1, 1,  1], [-1, 1,  1],
    ];
    // Edges (index pairs)
    const EDGES = [
        [0,1],[1,2],[2,3],[3,0],  // back face
        [4,5],[5,6],[6,7],[7,4],  // front face
        [0,4],[1,5],[2,6],[3,7],  // connecting
    ];
    // Face colors for the "phone" — front face highlighted
    const FACE_EDGES_FRONT = [[4,5],[5,6],[6,7],[7,4]];

    const COLORS = {
        edge: 'rgba(99, 102, 241, 0.5)',
        front: 'rgba(45, 212, 191, 0.8)',
        axis_x: '#f87171',
        axis_y: '#34d399',
        axis_z: '#6366f1',
    };

    function init(canvasId) {
        canvas = document.getElementById(canvasId);
        if (!canvas) return;
        ctx = canvas.getContext('2d');
        requestAnimationFrame(draw);
    }

    function setQuaternion(w, x, y, z) {
        quat = { w, x, y, z };
    }

    function quatRotate(q, v) {
        // Rotate vector v by quaternion q: q * v * q^-1
        const qv = { w: 0, x: v[0], y: v[1], z: v[2] };
        const qc = { w: q.w, x: -q.x, y: -q.y, z: -q.z };
        const t = quatMul(quatMul(q, qv), qc);
        return [t.x, t.y, t.z];
    }

    function quatMul(a, b) {
        return {
            w: a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
            x: a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
            y: a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
            z: a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        };
    }

    function project(v3, cx, cy, scale) {
        // Simple perspective projection
        const fov = 4;
        const z = v3[2] + fov;
        const factor = scale * fov / Math.max(z, 0.1);
        return [cx + v3[0] * factor, cy - v3[1] * factor];
    }

    function draw() {
        if (!ctx) { requestAnimationFrame(draw); return; }
        const W = canvas.width;
        const H = canvas.height;
        const cx = W / 2, cy = H / 2;
        const scale = Math.min(W, H) * 0.18;

        ctx.clearRect(0, 0, W, H);

        // Rotate all vertices
        const rotated = VERTS.map(v => quatRotate(quat, v));
        const projected = rotated.map(v => project(v, cx, cy, scale));

        // Draw edges
        ctx.lineWidth = 2;
        ctx.strokeStyle = COLORS.edge;
        ctx.beginPath();
        for (const [i, j] of EDGES) {
            ctx.moveTo(projected[i][0], projected[i][1]);
            ctx.lineTo(projected[j][0], projected[j][1]);
        }
        ctx.stroke();

        // Highlight front face
        ctx.lineWidth = 3;
        ctx.strokeStyle = COLORS.front;
        ctx.beginPath();
        for (const [i, j] of FACE_EDGES_FRONT) {
            ctx.moveTo(projected[i][0], projected[i][1]);
            ctx.lineTo(projected[j][0], projected[j][1]);
        }
        ctx.stroke();

        // Fill front face semi-transparent
        ctx.fillStyle = 'rgba(45, 212, 191, 0.08)';
        ctx.beginPath();
        ctx.moveTo(projected[4][0], projected[4][1]);
        ctx.lineTo(projected[5][0], projected[5][1]);
        ctx.lineTo(projected[6][0], projected[6][1]);
        ctx.lineTo(projected[7][0], projected[7][1]);
        ctx.closePath();
        ctx.fill();

        // Draw axes
        const axisLen = 1.8;
        const axes = [
            { v: [axisLen, 0, 0], c: COLORS.axis_x, l: 'X' },
            { v: [0, axisLen, 0], c: COLORS.axis_y, l: 'Y' },
            { v: [0, 0, axisLen], c: COLORS.axis_z, l: 'Z' },
        ];
        for (const ax of axes) {
            const rv = quatRotate(quat, ax.v);
            const p = project(rv, cx, cy, scale);
            ctx.strokeStyle = ax.c;
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(p[0], p[1]);
            ctx.stroke();
            ctx.fillStyle = ax.c;
            ctx.font = '11px Inter, sans-serif';
            ctx.fillText(ax.l, p[0] + 4, p[1] - 4);
        }

        requestAnimationFrame(draw);
    }

    return { init, setQuaternion };
})();
