#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QSlider>
#include <QFormLayout>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QTimer>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>
#include <QVector>
#include <QMap>

#include <boost/circular_buffer.hpp>

#include "qcustomplot.h"  // Include QCustomPlot library

const int MAX_DATA_SIZE = 1000; // Define maximum number of rows to store
// Define colors for the series
const QVector<QColor> lineColors = {
    Qt::black,
    Qt::red,
    Qt::green,
    Qt::blue,
    Qt::magenta,
    Qt::cyan,
    Qt::lightGray,
    Qt::darkGray
};
class SerialPlotter : public QMainWindow {
    Q_OBJECT

public:
    SerialPlotter(QWidget* parent = nullptr)
        : QMainWindow(parent),
        serialPort(new QSerialPort(this)),
        timer(new QTimer(this)),
        reconnectTimer(new QTimer(this)) {
        setWindowTitle("Serial Data Plotter");
        resize(800, 600);

        // Central widget
        centralWidget = new QWidget(this);
        setCentralWidget(centralWidget);

        // Layout
        layout = new QVBoxLayout(centralWidget);

        // Time range slider
        timeSlider = new QSlider(Qt::Horizontal);
        timeSlider->setMinimum(1);
        timeSlider->setMaximum(50);
        timeSlider->setValue(5);
        connect(timeSlider, &QSlider::valueChanged, this, &SerialPlotter::updateTimeRange);

        timeLabel = new QLabel("Time Range: " + QString::number(timeSlider->value()));
        layout->addWidget(timeSlider);
        layout->addWidget(timeLabel);

        // Port and baudrate selection layout
        QHBoxLayout* selectionLayout = new QHBoxLayout();
        layout->addLayout(selectionLayout);

        portLabel = new QLabel("Port:");
        selectionLayout->addWidget(portLabel);

        portSelector = new QComboBox();
        selectionLayout->addWidget(portSelector);

        baudLabel = new QLabel("Baudrate:");
        selectionLayout->addWidget(baudLabel);

        baudSelector = new QComboBox();
        baudSelector->addItems({"9600", "19200", "38400", "57600", "115200"});
        baudSelector->setCurrentText("115200");
        selectionLayout->addWidget(baudSelector);

        connectButton = new QPushButton("Connect");
        connect(connectButton, &QPushButton::clicked, this, &SerialPlotter::connectSerial);
        selectionLayout->addWidget(connectButton);

        infoLabel = new QLabel("Select port and baudrate to connect.");
        layout->addWidget(infoLabel);

        // Plot widget (using QCustomPlot)
        plotWidget = new QCustomPlot();
        plotWidget->addGraph(); // Add one graph for each series later
        layout->addWidget(plotWidget);

        // Start/Stop button
        startStopButton = new QPushButton("Start");
        startStopButton->setEnabled(false);
        connect(startStopButton, &QPushButton::clicked, this, &SerialPlotter::togglePloting);
        layout->addWidget(startStopButton);

        // Checkboxes for each data series
        checkboxLayout = new QFormLayout();
        layout->addLayout(checkboxLayout);

        // Initialize data
        resetData();
        detectSerialPorts();

        // interrupt mode to read data
        connect(serialPort, &QSerialPort::readyRead, this, &SerialPlotter::readSerialData);
        connect(serialPort, &QSerialPort::errorOccurred, this, &SerialPlotter::onSerialPortError);
        connect(timer, &QTimer::timeout, this,
            [this]() {
                checkConnection();
                updatePlot();
            });
        // serial port connection handling

    }    

private slots:
    void updateTimeRange() {
        int timeRangeSeconds = timeSlider->value();
        timeLabel->setText(QString("Time Range: %1s").arg(timeRangeSeconds));
        updatePlot();
    }

    void detectSerialPorts() {
        portSelector->clear();
        const auto ports = QSerialPortInfo::availablePorts();
        for (const QSerialPortInfo& port : ports) {
            portSelector->addItem(port.portName());
        }
        if (ports.empty()) {
            infoLabel->setText("No serial ports detected.");
        }
    }

    void connectSerial() {
        QString portName = portSelector->currentText();
        int baudRate = baudSelector->currentText().toInt();
        if (!portName.isEmpty()) {
            serialPort->setPortName(portName);
            serialPort->setBaudRate(baudRate);
            if (serialPort->open(QIODevice::ReadOnly)) {
                infoLabel->setText(QString("Connected to %1 at %2 baud.").arg(portName).arg(baudRate));
                startStopButton->setEnabled(true);
                // Create the required number of graphs dynamically
                plotWidget->clearGraphs(); // Clear existing graphs
                // Set up interactions (enable zooming and panning)
                plotWidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
                // Configure zooming to apply to both axes
                plotWidget->axisRect()->setRangeZoom(Qt::Horizontal | Qt::Vertical);
                plotWidget->xAxis->setLabel("Time (s)");
                plotWidget->yAxis->setLabel("Value");
                // Configure panning to apply to both axes
                plotWidget->axisRect()->setRangeDrag(Qt::Horizontal | Qt::Vertical);
                for (int i = 0; i < seriesCount; ++i) {
                    plotWidget->addGraph(); // Add graph for each series
                    QPen pen(lineColors[i % lineColors.size()]); // Cycle through colors if more than available
                    pen.setWidth(2); // Set line thickness
                    plotWidget->graph(i)->setPen(pen);
                }
                createSeriesCheckboxes();
            } else {
                infoLabel->setText("Failed to connect.");
            }
        } else {
            infoLabel->setText("No port selected.");
        }
    }

    // Periodic reconnection check
    void checkConnection() {
        if (!serialPort->isOpen()) {
            // Try to reconnect if the port is closed
            reconnectSerial();
        }
    }

    // Reconnect serial port
    void reconnectSerial() {
        QString portName = portSelector->currentText();

        // If the port is still disconnected, check every 5 seconds
        if (portDisconnected && !portName.isEmpty()) {
            serialPort->setPortName(portName);

            if (serialPort->open(QIODevice::ReadOnly)) {
                // Successfully reopened the port
                portDisconnected = false;
                infoLabel->setText(QString("Reconnected to %1 at %2 baud.").arg(portName).arg(baudSelector->currentText()));
                clearGraphData();
            }
        }
    }

    // Slot to handle serial port errors
    void onSerialPortError(QSerialPort::SerialPortError error) {
        if (error != QSerialPort::NoError && !portDisconnected) {
            qDebug() << "Serial port error occurred:" << error;

            // If the error is a disconnection or permission error, set the flag
            if (error == QSerialPort::ResourceError || error == QSerialPort::DeviceNotFoundError) {
                portDisconnected = true;
                infoLabel->setText("Serial port disconnected. Trying to reconnect...");
            }

            // Close the serial port if it was open
            if (serialPort->isOpen()) {
                serialPort->close();
            }
        }
    }

    void togglePloting() {
        if (!running) {
            if (serialPort->isOpen()) {
                running = true;
                startStopButton->setText("Stop");
                // Clear the data for all graphs without removing them
                clearGraphData();

                timer->start(1/replot_dt);
            } else {
                infoLabel->setText("Serial port not open.");
            }
        } else {
            running = false;
            startStopButton->setText("Start");
            timer->stop();
        }
    }

    void readSerialData() {
        static int cnt;
        if (!serialPort->canReadLine()) {
            return; // Skip if no data available
        }

        QByteArray line = serialPort->readLine().trimmed();
        QList<QByteArray> values = line.split(',');
        if (values.size() == seriesCount) {
            for (int i = 0; i < seriesCount; ++i) {
                yData[i].push_back(values[i].toDouble());
                if(i == 0) {
                    now = values[i].toDouble() * data_dt;
                    xData.push_back(now);
                    // qDebug() << ++cnt << " size " << xData.size();
                    infoLabel->setText(" now: " + QString::number(now, 'f', 3));
                    if (xData.size() > MAX_DATA_SIZE) xData.pop_front();
                }
                if (yData[i].size() > MAX_DATA_SIZE) {
                    yData[i].pop_front();
                }
                if(i > 0 && checkboxes[i]->isChecked())
                    plotWidget->graph(i)->addData(values[0].toDouble() * data_dt, values[i].toDouble());
            }
        } else {
            infoLabel->setText("Error parsing data: " + line);
        }
    }

    void updatePlot() {
        int timeRangeSeconds = timeSlider->value();
        int numPoints = timeRangeSeconds * (int)(1/data_dt); // 1 second = 20 data points (50ms interval)

        auto yMin = std::numeric_limits<double>::max();
        auto yMax = std::numeric_limits<double>::lowest();

        for (int i = 0; i < seriesCount; ++i) {
            if (checkboxes[i]->isChecked()) {
                if(now >= timeSlider->value())
                    plotWidget->graph(i)->data()->removeBefore(now - timeSlider->value());
                    plotWidget->graph(i)->rescaleValueAxis(true);

                for (auto it = plotWidget->graph(i)->data()->begin(); it != plotWidget->graph(i)->data()->end(); ++it) {
                    yMin = std::min(yMin, it->value);
                    yMax = std::max(yMax, it->value);
                }
            }
        }
        // Set the ranges for X and Y axes
        if(now < timeSlider->value()) {
            plotWidget->xAxis->setRange(0, now);
        } else {
            plotWidget->xAxis->setRange(now - timeSlider->value(), now);
        }
        if (yMin == yMax) {
            yMin -= yMin*0.01;
            yMax += yMax*0.01;
        }
        plotWidget->yAxis->setRange(yMin, yMax);

        plotWidget->replot();
    }

    void createSeriesCheckboxes() {
        for (auto i = 0; i < seriesCount; ++i) {
            auto name = seriesNames[i];
            // Update checkbox label with color
            QCheckBox* checkbox = new QCheckBox();
            QColor graphColor = lineColors[i % lineColors.size()];
            QLabel *label = new QLabel();
            QString labeltext = QString("<html><span style='color:%1;'>%2</span></html>")
                                .arg(graphColor.name()) // QColor::name() returns the color as #RRGGBB
                                .arg(name);
            label->setText(labeltext); // Set the styled text for QLabel
            checkbox->setChecked(true);
            checkbox->setObjectName(name);
            connect(checkbox, &QCheckBox::stateChanged, this, &SerialPlotter::clearPlot);
            checkboxes.push_back(checkbox);
            // Layout for checkbox and label
            QHBoxLayout *layout = new QHBoxLayout();
            layout->addWidget(checkbox);
            layout->addWidget(label);
            checkboxLayout->addRow(layout);
        }
    }

    void clearPlot(int state) {
        auto *checkBox = qobject_cast<QCheckBox *>(sender());
        if(checkBox && state == Qt::CheckState::Unchecked) {
            for(auto i = 0; i < seriesCount; ++i) {
                if (seriesNames[i] == checkBox->objectName())
                    plotWidget->graph(i)->data()->clear();
            }
        }
        plotWidget->replot();
    }
    void clearGraphData(){
        resetData();
        for (int i = 0; i < plotWidget->graphCount(); ++i) {
            plotWidget->graph(i)->data().clear();
        }

        // Replot to show the cleared state
        plotWidget->replot();
    }
private:
    void resetData() {
        xData.clear();
        if (!yData.empty()) {
            for(auto i = 0; i < seriesCount; ++i)
                yData[i].clear();
        } else {
            yData = QVector<QVector<double>>(seriesCount, QVector<double>());
        }
    }

    QWidget* centralWidget;
    QVBoxLayout* layout;

    QLabel* portLabel;
    QComboBox* portSelector;
    QLabel* baudLabel;
    QComboBox* baudSelector;
    QPushButton* connectButton;
    QLabel* infoLabel;

    QSlider* timeSlider;
    QLabel* timeLabel;

    QCustomPlot* plotWidget;

    QPushButton* startStopButton;
    QFormLayout* checkboxLayout;

    QSerialPort* serialPort;
    QTimer* timer;

    QVector<double> xData;
    QVector<QVector<double>> yData;
    QVector<QCheckBox*> checkboxes;

    QVector<QString> seriesNames = {"cnt",
                                    "mpu6050_ax",
                                    "mpu6050_ay",
                                    "mpu6050_az",
                                    "icm20948_ax",
                                    "icm20948_ay",
                                    "icm20948_az",
                                    "altitude"};
    int seriesCount = seriesNames.size();

    bool running = false;
    double now{0};
    double data_dt{0.05};
    double replot_dt{0.1};
    bool portDisconnected = false; // Flag to track if the port is disconnected
    QTimer *reconnectTimer; // Timer to check if the port is available
};

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    SerialPlotter plotter;
    plotter.show();
    return app.exec();
}

#include "main.moc"
