ros_deep_net_client
=====
`ros_deep_net_client` provides an interface for detecting objects in images and videos using `deep_object_detection` package.

Requirements
-----
* [deep_net package](https://github.com/hkaraoguz/deep_net.git)
* [ros_file_crawler package](https://github.com/hkaraoguz/ros_file_crawler.git)
* rospy message converter  (`sudo pip install rospy_message_converter`)

Example Usage for Images
-----
```
rosrun ros_deep_net_client image_object_detect_client.py /home/user/data/
```
This call will process all the `"jpg,png"` files with a confidence threshold of `0.8` located under the `/home/user/data` directory and output the results to the default output path `/home/user/image_object_detect_results/<datetime>` . `<datetime>` will be automatically generated during process.

### Advanced Usage
```
rosrun ros_deep_net_client image_object_detect_client.py <source_path> <confidence_threshold> <destination_path> <file_extensions> <excluded_words>
```
This call will process all the files with extensions `<file_extensions>` located under the `<source_path>` and outputs the results under `<destination_path>` based on `<confidence threshold>` . User can skip images with excluded words specified by parameter `<excuded_words>` .

Example Usage for Video URLs
-----
```
rosrun ros_deep_net_client url_object_detect_client.py <url> <confidence_threshold>
```
This call will process the video feed at the given `url` with a confidence threshold of `<confidence_threshold>` (default value is 0.8). When the video frame is on focus, if `q` is pressed, the node will quit. If `s` is pressed, the corresponding frame and the detected object information will be saved under `/home/user/url_object_detect_results/<datetime>` folder. `<datetime>` will be automatically generated when the user presses `s` for the first time.
