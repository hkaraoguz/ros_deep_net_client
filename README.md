ros_deep_net_client
=====
`ros_deep_net_client` provides an interface for processing images and videos and saving the detected objects by `deep_object_detection` package.

Requirements
-----
* deep_object_detection package (`https://github.com/hkaraoguz/deep_net.git`)
* ros_file_crawler package (`https://github.com/hkaraoguz/ros_file_crawler.git`)

Example Usage for Images
-----
```
rosrun ros_deep_net_client image_object_detect_client.py /home/user/data/
```
This call will process all the `"jpg,png"` files with a confidence threshold of `0.8` located under the `/home/user/data` directory and output the result to default output path `/home/user/image_object_detect_results` .

### Advanced Usage
```
rosrun ros_deep_net_client image_object_detect_client.py <source_path> <confidence_threshold> <destination_path> <file_extensions> <excluded_words>
```
This call will process all the files with extensions `<file_extensions>` located under the `<source_path>` and outputs the results under `<destination_path>` based on `<confidence threshold>` . User can skip images with excluded words specified by parameter `<excuded_words>` .
