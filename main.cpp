#include <QCoreApplication>
#include <QCommandLineParser>

#include <iostream>


#include "pathtracer.h"
#include "scene/scene.h"
#include "util/hdrloader.h"

#include <QImage>

#include "util/CS123Common.h"

#define CUSTOM_IMAGE_WIDTH 1024
#define CUSTOM_IMAGE_HEIGHT 1024

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("scene", "Scene file to be rendered");
    parser.addPositionalArgument("output", "Image file to write the rendered image to");
    parser.addPositionalArgument("samples", "Number of samples per pixel");
    parser.addPositionalArgument("hdr", "HDR light probe file");
    parser.addPositionalArgument("aperture", "Aperture size (0 is no DoF)");
    parser.addPositionalArgument("focal", "Focal length");

    parser.process(a);

    const QStringList args = parser.positionalArguments();
    if(args.size() != 6) {
        std::cerr << "Error: Wrong number of arguments" << std::endl;
        a.exit(1);
        return 1;
    }
    QString scenefile = args[0];
    QString output = args[1];
    int samples = args[2].toInt();
    QString hdr = args[3];
    float aperture = args[4].toFloat();
    float focal = args[5].toFloat();

    QImage image(CUSTOM_IMAGE_WIDTH, CUSTOM_IMAGE_HEIGHT, QImage::Format_RGB32);

    Scene *scene;
    if(!Scene::load(scenefile, &scene)) {
        std::cerr << "Error parsing scene file " << scenefile.toStdString() << std::endl;
        a.exit(1);
        return 1;
    }

    HDRLoaderResult hdrResult;
    hdrResult.width = 0;
    hdrResult.height = 0;
    if (!HDRLoader::load(hdr.toUtf8(), hdrResult)) {
        std::cerr << "HDR file not found, using black environment" << std::endl;
    }

    PathTracer tracer(CUSTOM_IMAGE_WIDTH, CUSTOM_IMAGE_HEIGHT, hdrResult, aperture, focal);

    QRgb *data = reinterpret_cast<QRgb *>(image.bits());

    tracer.traceScene(data, *scene, samples);
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
