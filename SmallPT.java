import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class SmallPT {

    private static long state0 = (long)(Math.random() * Long.MAX_VALUE);
    private static long state1 = (long)(Math.random() * Long.MAX_VALUE);

    private static long randomLong() {
        long s1 = state0;
        long s0 = state1;
        state0 = s0;
        s1 ^= s1 << 23;
        state1 = s1 ^ s0 ^ s1 >>> 17 ^ s0 >>> 26;
        return state1 + s0;
    }

    private static double erand48() {
        return 0.5 + randomLong() * 0.5 / Long.MAX_VALUE;
    }

    private static double clamp(double x) {
        return x < 0 ? 0 : x > 1 ? 1 : x;
    }

    private static int toInt(double x) {
        return (int)(Math.pow(clamp(x), 1.0 / 2.2) * 255.0 + 0.5);
    }

    private static void saveImage(int width, int height, Vector[][] colors) throws IOException {
        final BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                Vector vec = colors[y][x];
                final int red = toInt(vec.x);
                final int grn = toInt(vec.y);
                final int blu = toInt(vec.z);
                int rgb = new Color(red, grn, blu).getRGB();
                image.setRGB(x, height - y - 1, rgb);
            }
        }
        ImageIO.write(image, "png", new File("image.png"));
    }

    private final Sphere spheres[] = {
            new Sphere(1e5, new Vector(1e5 + 1, 40.8, 81.6), new Vector(0, 0, 0), new Vector(.75, .25, .25), ReflectanceType.DIFF),
            new Sphere(1e5, new Vector(-1e5 + 99, 40.8, 81.6), new Vector(0, 0, 0), new Vector(.25, .25, .75), ReflectanceType.DIFF),
            new Sphere(1e5, new Vector(50, 40.8, 1e5), new Vector(0, 0, 0), new Vector(.75, .75, .75), ReflectanceType.DIFF),
            new Sphere(1e5, new Vector(50, 40.8, -1e5 + 170), new Vector(0, 0, 0), new Vector(0, 0, 0), ReflectanceType.DIFF),
            new Sphere(1e5, new Vector(50, 1e5, 81.6), new Vector(0, 0, 0), new Vector(.75, .75, .75), ReflectanceType.DIFF),
            new Sphere(1e5, new Vector(50, -1e5 + 81.6, 81.6), new Vector(0, 0, 0), new Vector(.75, .75, .75), ReflectanceType.DIFF),
            new Sphere(16.5, new Vector(27, 16.5, 47), new Vector(0, 0, 0), new Vector(0.999, 0.999, 0.999), ReflectanceType.SPEC),
            new Sphere(16.5, new Vector(73, 16.5, 78), new Vector(0, 0, 0), new Vector(0.999, 0.999, 0.999), ReflectanceType.SPEC),
            new Sphere(600, new Vector(50, 681.6 - .27, 81.6), new Vector(12, 12, 12), new Vector(0, 0, 0), ReflectanceType.DIFF)
    };

    private IntersectionResult intersect(Ray r) {
        Sphere hit = null;
        double nearest = Double.POSITIVE_INFINITY;
        for (Sphere sphere : spheres) {
            double dist = sphere.intersect(r);
            if (dist > 0.0 && dist < nearest) {
                hit = sphere;
                nearest = dist;
            }
        }
        return new IntersectionResult(hit, nearest);
    }

    private Vector radiance(Ray ray, int depth) {
        IntersectionResult result = intersect(ray);
        Sphere obj = result.sphere;
        if (obj == null) {
            return Vector.ZERO;
        }

        Vector x = ray.origin.add(ray.direction.mul(result.distance));
        Vector n = x.sub(obj.position).norm();
        Vector nl = n.dot(ray.direction) < 0.0 ? n : n.mul(-1.0);
        Vector f = obj.color;
        double p = f.x > f.y && f.x > f.z ? f.x : f.y > f.z ? f.y : f.z;
        if (++depth > 5) {
            if (erand48() < p) {
                f = f.mul(1.0 / p);
            } else {
                return obj.emission;
            }
        }

        if (obj.type == ReflectanceType.DIFF) {
            double r1 = 2.0 * Math.PI * erand48();
            double r2 = erand48();
            double r2s = Math.sqrt(r2);
            Vector u = (Math.abs(nl.x) > 0.1 ? Vector.UP : Vector.RIGHT).cross(nl).norm();
            Vector d = u.mul(Math.cos(r1) * r2s).add(nl.cross(u).mul(Math.sin(r1) * r2s)).add(nl.mul(Math.sqrt(1 - r2))).norm();
            return obj.emission.add(f.mul(radiance(new Ray(x, d), depth)));
        } else if (obj.type == ReflectanceType.SPEC) {
            Vector d = ray.direction.sub(n.mul(2.0 * n.dot(ray.direction)));
            return obj.emission.add(f.mul(radiance(new Ray(x, d), depth)));
        }

        Ray reflRay = new Ray(x, ray.direction.sub(n.mul(2 * n.dot(ray.direction))));
        boolean into = n.dot(nl) > 0;
        double nc = 1, nt = 1.5, nnt = into ? nc / nt : nt / nc, ddn = ray.direction.dot(nl), cos2t;
        if ((cos2t = 1 - nnt * nnt * (1 - ddn * ddn)) < 0) {
            return obj.emission.add(f.mul(radiance(reflRay, depth)));
        }

        Vector tdir = ((ray.direction.mul(nnt)).sub(n.mul((into ? 1 : -1) * (ddn * nnt + Math.sqrt(cos2t))))).norm();
        double a = nt - nc, b = nt + nc, R0 = a * a / (b * b), c = 1 - (into ? -ddn : tdir.dot(n));
        double Re = R0 + (1 - R0) * c * c * c * c * c, Tr = 1 - Re, P = .25 + .5 * Re, RP = Re / P, TP = Tr / (1 - P);
        return obj.emission.add(f.mul(depth > 2 ? (erand48() < P ? radiance(reflRay, depth).mul(RP) : radiance(new Ray(x, tdir), depth).mul(TP)) : (radiance(reflRay, depth).mul(Re)).add(radiance(new Ray(x, tdir), depth).mul(Tr))));
    }

    private void run(String[] args) throws IOException, InterruptedException {
        int w = 256;
        int h = 256;
        int samples = args.length > 0 ? Integer.parseInt(args[0]) / 4 : 1;
        Ray cam = new Ray(new Vector(50, 52, 295.6), new Vector(0, -0.042612, -1).norm());
        Vector cx = new Vector(w * .5135 / h, 0, 0);
        Vector cy = (cx.cross(cam.direction)).norm().mul(.5135);
        Vector[][] c = new Vector[h][w];
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                c[y][x] = new Vector();
            }
        }

        ExecutorService service = Executors.newScheduledThreadPool(Runtime.getRuntime().availableProcessors());

        for (int y = 0; y < h; y++) {
            final int ty = y;
            service.execute(() -> {
                System.out.printf("\r Rendering (%d spp) %5.2f%%", samples * 4, 100.0 * ty / (h - 1));
                for (int x = 0; x < w; x++) {
                    for (int sy = 0; sy < 2; sy++) {
                        for (int sx = 0; sx < 2; sx++) {
                            Vector r = new Vector();
                            for (int s = 0; s < samples; s++) {
                                double r1 = 2.0 * erand48();
                                double dx = r1 < 1.0 ? Math.sqrt(r1) - 1.0 : 1.0 - Math.sqrt(2.0 - r1);
                                double r2 = 2.0 * erand48();
                                double dy = r2 < 1.0 ? Math.sqrt(r2) - 1.0 : 1.0 - Math.sqrt(2.0 - r2);
                                Vector d = (cx.mul(((sx + 0.5 + dx) / 2.0 + x) / w - 0.5)).add(cy.mul(((sy + 0.5 + dy) / 2.0 + ty) / h - 0.5)).add(cam.direction);
                                r = r.add(radiance(new Ray(cam.origin.add(d.mul(140.0)), d.norm()), 0).mul((1.0 / samples)));
                            }
                            c[ty][x] = c[ty][x].add(new Vector(clamp(r.x), clamp(r.y), clamp(r.z)).mul(0.25));
                        }
                    }
                }
            });
        }

        service.shutdown();
        service.awaitTermination(1, TimeUnit.HOURS);

        saveImage(w, h, c);
    }

    public static void main(String[] args) throws IOException, InterruptedException {
        new SmallPT().run(args);
    }
}

final class IntersectionResult {
    final Sphere sphere;
    final double distance;

    IntersectionResult(Sphere sphere, double distance) {
        this.sphere = sphere;
        this.distance = distance;
    }
}

final class Ray {
    final Vector origin;
    final Vector direction;

    Ray(final Vector origin, final Vector direction) {
        this.origin = origin;
        this.direction = direction;
    }
}

enum ReflectanceType {
    DIFF,
    SPEC,
    REFR
}

final class Sphere {
    private static final double EPSILON = 0.0001;

    final double radius;
    final Vector position;
    final Vector emission;
    final Vector color;
    final ReflectanceType type;

    Sphere(double radius, Vector position, Vector emission, Vector color, ReflectanceType type) {
        this.radius = radius;
        this.position = position;
        this.emission = emission;
        this.color = color;
        this.type = type;
    }

    double intersect(final Ray r) {
        Vector op = position.sub(r.origin);
        double b = op.dot(r.direction);
        double det = b * b - op.dot(op) + radius * radius;
        if (det < 0.0) {
            return 0.0;
        } else {
            det = Math.sqrt(det);
        }
        double t;
        return (t = b - det) > EPSILON ? t : (t = b + det) > EPSILON ? t : 0;
    }
}

final class Vector {
    static final Vector ZERO = new Vector(0, 0, 0);
    static final Vector UP = new Vector(0, 1, 0);
    static final Vector RIGHT = new Vector(1, 0, 0);

    final double x;
    final double y;
    final double z;

    Vector() {
        this(0.0, 0.0, 0.0);
    }

    Vector(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    double length() {
        return Math.sqrt(this.dot(this));
    }

    double dot(final Vector b) {
        return x * b.x + y * b.y + z * b.z;
    }

    Vector add(final Vector b) {
        return new Vector(x + b.x, y + b.y, z + b.z);
    }

    Vector sub(final Vector b) {
        return new Vector(x - b.x, y - b.y, z - b.z);
    }

    Vector mul(double b) {
        return new Vector(x * b, y * b, z * b);
    }

    Vector mul(final Vector b) {
        return new Vector(x * b.x, y * b.y, z * b.z);
    }

    Vector norm() {
        return this.mul(1.0 / length());
    }

    Vector cross(Vector b) {
        return new Vector(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
    }
}
