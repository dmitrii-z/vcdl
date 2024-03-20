#define NOMINMAX

#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <filesystem>
#include <fstream>
#include "argparse.hpp"
#include "SilAdapter/SilAdapter.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <libssh/sftp.h>

using namespace std::chrono_literals;

#define MAX_XFER_BUF_SIZE 16384
#define O_RDONLY         00
#define O_WRONLY         01
#define O_RDWR           02
char sftp_buffer[MAX_XFER_BUF_SIZE];

class SftpDownloader {
public:
    SftpDownloader() {}
    ~SftpDownloader() { if (connected()) { disconnect(); } };
    bool connect(const std::string& host, const std::string& login, const std::string& password);
    void disconnect();
    bool download(const std::string& filename, const std::string& remote_path, const std::string& local_path);
    bool connected() const { return _ssh_session != nullptr && _sftp_session != nullptr; }
private:
    ssh_session _ssh_session = nullptr;
    sftp_session _sftp_session = nullptr;

};

bool SftpDownloader::connect(const std::string& host, const std::string& login, const std::string& password) {
    if (connected()) {
        disconnect();
    }
    int rc;

    // Open session and set options
    _ssh_session = ssh_new();
    if (_ssh_session == nullptr) {
        return false;
    }
    ssh_options_set(_ssh_session, SSH_OPTIONS_HOST, host.c_str());

    // Connect to server
    rc = ssh_connect(_ssh_session);
    if (rc != SSH_OK)
    {
        fprintf(stderr, "Error connecting to localhost: %s\n", ssh_get_error(_ssh_session));
        disconnect();
        return false;
    }

    // Verify the server's identity
    // For the source code of verify_knownhost(), check previous example
    /*if (verify_knownhost(_ssh_session) < 0)
    {
        fprintf(stderr, "Host verification failed\n");
        disconnect();
        return false;
    }*/

    // Authenticate ourselves
    rc = ssh_userauth_password(_ssh_session, login.c_str(), password.c_str());
    if (rc != SSH_AUTH_SUCCESS)
    {
        fprintf(stderr, "Error authenticating with password: %s\n", ssh_get_error(_ssh_session));
        disconnect();
        return false;
    }

    _sftp_session = sftp_new(_ssh_session);
    if (_sftp_session == nullptr)
    {
        fprintf(stderr, "Error allocating SFTP session: %s\n", ssh_get_error(_ssh_session));
        disconnect();
        return false;
    }

    rc = sftp_init(_sftp_session);
    if (rc != SSH_OK)
    {
        fprintf(stderr, "Error initializing SFTP session: code %d.\n", sftp_get_error(_sftp_session));
        disconnect();
        return false;
    }

    return true;
}

void SftpDownloader::disconnect() {
    if (_sftp_session != nullptr) {
        sftp_free(_sftp_session);
        _sftp_session = nullptr;
    }
    if (_ssh_session != nullptr) {
        if (ssh_is_connected(_ssh_session)) {
            ssh_disconnect(_ssh_session);
        }
        ssh_free(_ssh_session);
        _ssh_session = nullptr;
    }
}

bool SftpDownloader::download(const std::string& filename, const std::string& remote_path, const std::string& local_path) {
    if (!connected()) {
        return false;
    }

    int access_type;
    sftp_file file;
    size_t nbytes;
    int rc;
    
    access_type = O_RDONLY;
    file = sftp_open(_sftp_session, (remote_path + '/' + filename).c_str(),
        access_type, 0);
    if (file == NULL) {
        fprintf(stderr, "Can't open file for reading: %s\n", ssh_get_error(_sftp_session));
        return false;
    }

    std::ofstream fs;
    fs.open((std::filesystem::path(local_path) / filename).string(), std::ios::binary);
    if (!fs.is_open()) {
        fprintf(stderr, "Can't open file for writing\n");
        return false;
    }

    for (;;) {
        nbytes = sftp_read(file, sftp_buffer, sizeof(sftp_buffer));
        if (nbytes == 0) {
            break; // EOF
        }
        else if (nbytes < 0) {
            fprintf(stderr, "Error while reading file: %s\n", ssh_get_error(_sftp_session));
            sftp_close(file);
            return false;
        }
        std::cout << '.';
        fs.write(sftp_buffer, nbytes);
    }
    std::cout << '\n';

    rc = sftp_close(file);
    if (rc != SSH_OK) {
        fprintf(stderr, "Can't close the read file: %s\n", ssh_get_error(_sftp_session));
        return false;
    }

    return true;
}


//#define MOCK
#ifdef MOCK
struct MockHandshake {
    bool IsActive = false;
    unsigned int FrameIndexRequested = 0;
    unsigned int FrameIndexWrite = 0;
};

struct MockCameraInfo {
    double FrontTimestamp;
    double LeftTimestamp;
    double RearTimestamp;
    double RightTimestamp;
};

auto& sync = MockHandshake();
auto& camInfo = MockCameraInfo();
typedef MockCameraInfo CamInfo;

double Timestamps[] = {
#include "timestamps.inc"
};

void mockInit() {
    sync.IsActive = true;
    sync.FrameIndexWrite = 0;
    camInfo.FrontTimestamp = Timestamps[0];
    camInfo.LeftTimestamp = Timestamps[0];
    camInfo.RearTimestamp = Timestamps[0];
    camInfo.RightTimestamp = Timestamps[0];
}

void mockForward() {
    if (sync.FrameIndexWrite < 1500) {
        sync.FrameIndexWrite++;
        camInfo.FrontTimestamp = Timestamps[sync.FrameIndexWrite];
        camInfo.LeftTimestamp = Timestamps[sync.FrameIndexWrite];
        camInfo.RearTimestamp = Timestamps[sync.FrameIndexWrite];
        camInfo.RightTimestamp = Timestamps[sync.FrameIndexWrite];
    }
    else {
        sync.IsActive = false;
    }
}
#else

#define mockInit()
#define mockForward()

typedef Vector::CANoe::TypeLib::DistributedObject<TestSiLAdapter::ISync> Sync;
typedef Vector::CANoe::TypeLib::DistributedObject<TestSiLAdapter::ICameraInfo> CamInfo;

#endif

cv::Mat img;
cv::Mat fronts, rights, rears, lefts;

bool showFrames(const cv::Mat& front, const cv::Mat& right, const cv::Mat& rear, const cv::Mat& left, int milliseconds, int downscale, bool panoramic) {
    if (downscale > 1) {
        double factor = 1.0 / downscale;
        resize(front, fronts, cv::Size(), factor, factor);
        resize(right, rights, cv::Size(), factor, factor);
        resize(rear, rears, cv::Size(), factor, factor);
        resize(left, lefts, cv::Size(), factor, factor);
    }
    const cv::Mat& _front = downscale > 1 ? fronts : front;
    const cv::Mat& _right = downscale > 1 ? rights : right;
    const cv::Mat& _rear = downscale > 1 ? rears : rear;
    const cv::Mat& _left = downscale > 1 ? lefts : left;

    if (img.empty()) {
        if (panoramic) {
            int w = _front.cols + _right.cols + _rear.cols + _left.cols;
            int h = std::max(std::max(_front.rows, _right.rows), std::max(_rear.rows, _left.rows));
            img = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
        }
        else {
            int w = std::max(_front.cols + _right.cols, _rear.cols + _left.cols);
            int h = std::max(_front.rows + _right.rows, _rear.rows + _left.rows);
            img = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
        }
    }

    if (panoramic) {
        // panoramic
        _left.copyTo(img(cv::Rect(0, 0, _left.cols, _left.rows)));
        _front.copyTo(img(cv::Rect(_left.cols, 0, _front.cols, _front.rows)));
        _right.copyTo(img(cv::Rect(_left.cols + _front.cols, 0, _right.cols, _right.rows)));
        _rear.copyTo(img(cv::Rect(_left.cols + _front.cols + _right.cols, 0, _rear.cols, _rear.rows)));

        putText(img, std::to_string(milliseconds), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(200, 0, 0), 2);
        imshow("video", img);
    }
    else {
        // box
        _front.copyTo(img(cv::Rect(0, 0, _front.cols, _front.rows)));
        _right.copyTo(img(cv::Rect(std::max(_front.cols, _rear.cols), 0, _right.cols, _right.rows)));
        _rear.copyTo(img(cv::Rect(0, std::max(_front.rows, _right.rows), _rear.cols, _rear.rows)));
        _left.copyTo(img(cv::Rect(std::max(_front.cols, _rear.cols), std::max(_front.rows, _right.rows), _left.cols, _left.rows)));

        putText(img, std::to_string(milliseconds), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(200, 0, 0), 2);
        imshow("video", img);
    }

    auto key = cv::waitKey(1);
    if (key == 32) {
        key = cv::waitKey(0);
    }

    return key != 27;
}


std::pair<std::vector<int>, int> getFrameImageNames(const std::string& path) {
    std::pair<std::vector<int>, int> result;
    for (const auto& entry : std::filesystem::directory_iterator(path))
    {
        if (entry.is_directory()) {
            std::string filename = entry.path().filename().string();
            if (std::stoi(filename) > 0) {
                result.second = (int)filename.size();
            }
            result.first.push_back(std::stoi(filename));
        }
    }
    std::sort(result.first.begin(), result.first.end());        
    return result;
}

std::string img_file_names[] = {"front.png", "left.png", "rear.png", "right.png"};

long long deltaMSec(std::chrono::duration<double> base, std::chrono::duration<double> time) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(time - base).count();
}

std::vector<cv::Mat> loadFrames(std::chrono::duration<double> baseTime[4], std::chrono::duration<double> time[4], const std::string& img_path, const std::vector<int>& frames, int padding) {
    std::vector<cv::Mat> images(4);
    for (int i = 0; i < 4; ++i) {
        long long delta = deltaMSec(baseTime[i], time[i]);
        auto it_frame = std::lower_bound(frames.begin(), frames.end(), delta);
        std::string subdir = std::to_string(*it_frame);
        subdir.insert(subdir.begin(), padding - subdir.length(), '0');
        auto path = std::filesystem::path(img_path) / subdir / img_file_names[i];
        images[i] = std::filesystem::exists(path) ? cv::imread(path.string()) : cv::Mat();
    }
    return images;
}

std::vector<cv::Mat> getFramesFromStreams(std::chrono::duration<double> baseTime[4], std::chrono::duration<double> currentTime[4], int frameTimeMSec[4], std::vector<cv::VideoCapture>& cap) {
    std::vector<cv::Mat> images(4);
    for (int i = 0; i < 4; ++i) {
        double frame_length_ms = 1000.0 / cap[i].get(cv::CAP_PROP_FPS);
        auto currentMSec = deltaMSec(baseTime[i], currentTime[i]);
        //static const double epsilon_ms = 20.0;
        if (abs(frameTimeMSec[i] - currentMSec) > frame_length_ms) {
            std::cout << "Synchronizing stream positions\n";
            cap[i].set(cv::CAP_PROP_POS_MSEC, (double)currentMSec);
            frameTimeMSec[i] = (int)currentMSec;
        }
        std::cout << "Getting frame\n";
        cap[i] >> images[i];
        std::cout << "Done\n";
        frameTimeMSec[i] += (int)frame_length_ms;
    }
    return images;
}

std::vector<std::vector<std::chrono::duration<double>>> loadTimestamps(const std::string& filename) {
    std::vector<std::vector<std::chrono::duration<double>>> result;
    std::ifstream csvfile;
    csvfile.open(filename);
    std::string line;
    while (std::getline(csvfile, line))
    {
        std::istringstream iss(line);
        std::vector<std::chrono::duration<double>> ts;
        for (int i = 0; i < 4; ++i) {
            std::string _;
            std::getline(iss, _, ',');
            std::string d;
            std::getline(iss, d, ',');
            try {
                ts.push_back(std::chrono::duration<double>(stod(d)));
            }
            catch (...) {
                break;
            }
        }
        if (ts.size() == 4) {
            result.push_back(ts);
        }
    }
    return result;
}

void assignTimestamps(CamInfo& camInfo, std::chrono::duration<double> times[4]) {
    times[0] = std::chrono::duration<double>(camInfo.FrontTimestamp);
    times[1] = std::chrono::duration<double>(camInfo.LeftTimestamp);
    times[2] = std::chrono::duration<double>(camInfo.RearTimestamp);
    times[3] = std::chrono::duration<double>(camInfo.RightTimestamp);
}

int main(int argc, char** argv)
{
    argparse::ArgumentParser program("parking");
    program.add_argument("-b").flag();
    program.add_argument("-p").flag();
    program.add_argument("--downscale").default_value(1).scan<'i', int>();
    program.add_argument("--remotepath").default_value("");
    program.add_argument("--sshhost").default_value("");
    program.add_argument("--sshlogin").default_value("");
    program.add_argument("--sshpassword").default_value("");
    program.add_argument("src").default_value("");

    try {
        program.parse_args(argc, argv); // Example: ./main -abc 1.95 2.47
    }
    catch (const std::exception& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    auto src = program.get<std::string>("src");
    bool syncOnly = src.empty();
    if (syncOnly) {
        std::cout << "Testing synchronization, video input disabled\n";
    }

    auto remotepath =  program.get<std::string>("remotepath");
    auto sshhost = program.get<std::string>("sshhost");
    auto sshlogin = program.get<std::string>("sshlogin");
    auto sshpassword = program.get<std::string>("sshpassword");
    if (remotepath != "") {
        if (sshhost == "" || sshlogin == "" || sshpassword == "") {
            std::cout << "sshhost, sshlogin and sshpassword required for SFTP session\n";
            return 1;
        }
        SftpDownloader d;
        if (!d.connect(sshhost, sshlogin, sshpassword)) {
            std::cout << "SSH connection error\n";
            return 1;
        }
        std::vector<std::string> files = { "front.mp4", "left.mp4", "rear.mp4", "right.mp4", "calibration.json", "synchronize_all_to_camera_SVM_front.csv" };
        for (int i = 0; i < files.size(); ++i) {
            if (!d.download(files[i], remotepath, src)) {
                std::cout << "SSH downloading error\n";
                return 1;
            }
        }
    }

    int downscale = program.get<int>("--downscale");
    bool display_box = program.get<bool>("-b");
    bool display_panoramic = program.get<bool>("-p");

    std::vector<cv::VideoCapture> cap{
                            cv::VideoCapture((std::filesystem::path(src) / "front.mp4").string()),
                            cv::VideoCapture((std::filesystem::path(src) / "left.mp4").string()),
                            cv::VideoCapture((std::filesystem::path(src) / "rear.mp4").string()),
                            cv::VideoCapture((std::filesystem::path(src) / "right.mp4").string()) };
    if (!syncOnly && !all_of(cap.cbegin(), cap.cend(), [](auto& c) { return c.isOpened(); })) {
        std::cout << "Can't open one of the video files\n";
        return 1;
    }

    Vector::CANoe::SilAdapter::Connect();

#ifndef MOCK
    auto& sync = TestSiLAdapter::Sync;
    auto& camInfo = TestSiLAdapter::CameraInfo;
#endif

    std::chrono::duration<double> baseTime[4] = { std::chrono::duration<double>(), std::chrono::duration<double>(), std::chrono::duration<double>(), std::chrono::duration<double>() };
    std::chrono::duration<double> time[4] = { std::chrono::duration<double>(), std::chrono::duration<double>(), std::chrono::duration<double>(), std::chrono::duration<double>() };
    int frameTimeMSec[4];
    std::cout << "Application ready\n";
    
    mockInit();

    cv::Mat f;
    cap[0] >> f;

    while (true) {
        if (sync.IsActive)
        {
            if (sync.FrameIndexWrite == 0) {
                assignTimestamps(camInfo, baseTime);
                assignTimestamps(camInfo, time);
                std::cout << "Canoe restarted\n";
                frameTimeMSec[0] = frameTimeMSec[1] = frameTimeMSec[2] = frameTimeMSec[3] = 0;
                std::cout << "Timestamp: " << camInfo.FrontTimestamp << " (" << int(deltaMSec(baseTime[0], time[0]) / 1000) << "sec)\n";
                if (!syncOnly) {
                    auto images = getFramesFromStreams(baseTime, time, frameTimeMSec, cap);
                    showFrames(images[0], images[3], images[2], images[1], (int)deltaMSec(baseTime[0], time[0]), downscale, display_panoramic);
                }

                sync.FrameIndexRequested = 0;
                std::this_thread::sleep_for(10ms);
                sync.FrameIndexRequested = 1;
                mockForward();
            }
            else if (sync.FrameIndexWrite == sync.FrameIndexRequested) {
                assignTimestamps(camInfo, time);
                std::cout << "Timestamp: " << camInfo.FrontTimestamp << " (" << int(deltaMSec(baseTime[0], time[0]) / 1000) << "sec)\n";
                if (!syncOnly) {
                    auto images = getFramesFromStreams(baseTime, time, frameTimeMSec, cap);
                    showFrames(images[0], images[3], images[2], images[1], (int)deltaMSec(baseTime[0], time[0]), downscale, display_panoramic);
                }
                sync.FrameIndexRequested = sync.FrameIndexRequested + 1;
                mockForward();
            }
            else if (sync.FrameIndexWrite + 1 == sync.FrameIndexRequested) {
                std::cout << "Iteration requested, waiting for sync\n";
                std::this_thread::sleep_for(10ms);
            }
            else {
                std::cout << "Unexpected sync state: write " << sync.FrameIndexWrite << " rq " << sync.FrameIndexRequested << "\n";
                sync.FrameIndexRequested = sync.FrameIndexWrite + 1;
            }
        }
        else {
            std::cout << "Waiting for activation\n";
            std::this_thread::sleep_for(100ms);
        }
        
        /*if (getche() == 27) {
            break;
        }*/
        std::cout << "End loop iteration\n";
    }

    std::cout << "Exiting\n";
    Vector::CANoe::SilAdapter::Disconnect();

    return 0;
}
