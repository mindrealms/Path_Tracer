#include <QCoreApplication>
#include <QCommandLineParser>

#include <iostream>

#include "pathtracer.h"
#include "scene/scene.h"

#include <QImage>

#include "util/CS123Common.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("scene", "Scene file to be rendered");
    parser.addPositionalArgument("output", "Image file to write the rendered image to");

    parser.process(a);

    const QStringList args = parser.positionalArguments();
    if(args.size() != 7) {
        std::cerr << "Error: Wrong number of arguments" << std::endl;
        a.exit(1);
        return 1;
    }
    QString scenefile = args[0];
    QString output = args[1];
    QString lightprobe = args[2]; //light probe file
    int samples = args[3].toInt();
    QString dof = args[4];
    float focal_l = args[5].toFloat(); //distance at which objects are in focus (+1 to account for image plane location)
    float aperture = args[6].toFloat(); //radius of disc

    if (lightprobe[0].isDigit() == 0) {
        std::cout << "No light probe to be used" << std::endl;
    }

    bool dof_mode;
    if (dof == "on") {
        dof_mode = true;
    } else if (dof == "off") {
        dof_mode = false;
    } else {
        std::cerr << "Error: incorrect DOF mode argument" << std::endl;
        a.exit(1);
        return 1;
    }

    if (focal_l < 0) {
        std::cerr << "Error: invalid focal length value" << std::endl;
        a.exit(1);
        return 1;
    }

    if (aperture < 0) {
        std::cerr << "Error: invalid aperture value" << std::endl;
        a.exit(1);
        return 1;
    }

    QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB32);

    Scene *scene;
    if(!Scene::load(scenefile, &scene)) {
        std::cerr << "Error parsing scene file " << scenefile.toStdString() << std::endl;
        a.exit(1);
        return 1;
    }

    PathTracer tracer(IMAGE_WIDTH, IMAGE_HEIGHT, samples, lightprobe, dof_mode, focal_l + 1.f, aperture);

    QRgb *data = reinterpret_cast<QRgb *>(image.bits());

    tracer.traceScene(data, *scene);
    delete scene;

    bool success = image.save(output);
    if(!success) {
        success = image.save(output, "PNG");
    }
    if(success) {
        std::cout << "Wrote rendered image to " << output.toStdString() << std::endl;
    } else {
        std::cerr << "Error: failed to write image to " << output.toStdString() << std::endl;
    }
    a.exit();
}
