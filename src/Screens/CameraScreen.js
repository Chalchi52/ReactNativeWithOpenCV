import React, { Component } from "react";
import {
  AppRegistry,
  View,
  Text,
  Platform,
  Image,
  TouchableOpacity
} from "react-native";
import { RNCamera as Camera } from "react-native-camera";
import Toast, { DURATION } from "react-native-easy-toast";

import styles from "../Styles/Screens/CameraScreen";
import OpenCV from "../NativeModules/OpenCV";
import CircleWithinCircle from "../assets/svg/CircleWithinCircle";

export default class CameraScreen extends Component {
  constructor(props) {
    super(props);

    this.takePicture = this.takePicture.bind(this);
    this.checkForBlurryImage = this.checkForBlurryImage.bind(this);
    this.proceedWithCheckingBlurryImage = this.proceedWithCheckingBlurryImage.bind(
      this
    );
    this.repeatPhoto = this.repeatPhoto.bind(this);
    this.usePhoto = this.usePhoto.bind(this);
  }

  state = {
    cameraPermission: false,
    photoAsBase64: {
      content: "",
      isPhotoPreview: false,
      photoPath: ""
    }
  };

  checkForBlurryImage(imageAsBase64) {
    return new Promise((resolve, reject) => {
      if (Platform.OS === "android") {
        OpenCV.checkForBlurryImage(
          imageAsBase64,
          error => {
            // error handling
          },
          msg => {
            resolve(msg);
          }
        );
      } else {
        OpenCV.checkForBlurryImage(imageAsBase64, (error, dataArray) => {
          resolve(dataArray[0]);
        });
      }
    });
  }

  proceedWithCheckingBlurryImage() {
    const { content, photoPath } = this.state.photoAsBase64;

    this.checkForBlurryImage(content)
      .then(blurryPhoto => {
        // if (blurryPhoto) {
        //   this.refs.toast.show("Photo is blurred!", DURATION.FOREVER);
        //   return this.repeatPhoto();
        // }
        this.refs.toast.show("Photo is clear!", DURATION.LENGTH_SHORT);
        this.setState({
          photoAsBase64: {
            ...this.state.photoAsBase64,
            isPhotoPreview: true,
            photoPath
          }
        });
      })
      .catch(err => {
        console.log("err", err);
      });
  }

  async takePicture() {
    if (this.camera) {
      const options = { quality: 0.5, base64: true };
      const data = await this.camera.takePictureAsync(options);
      this.setState({
        ...this.state,
        photoAsBase64: {
          content: data.base64,
          isPhotoPreview: false,
          photoPath: data.uri
        }
      });
      this.proceedWithCheckingBlurryImage();
    }
  }

  repeatPhoto() {
    this.setState({
      ...this.state,
      photoAsBase64: {
        content: "",
        isPhotoPreview: false,
        photoPath: ""
      }
    });
  }

  usePhoto(photoPath) {
    // do something, e.g. navigate
    if (Platform.OS === "android") {
      OpenCV.rotateImage(photoPath)
        .then(result => {
          console.log(result);
          this.repeatPhoto();
          this.refs.toast.show("Photo rotated!", DURATION.LENGTH_SHORT);
        })
        .catch(error => {
          console.log(error);
          this.refs.toast.show(""+error, DURATION.LENGTH_SHORT);
        });
    } else {
      OpenCV.rotateImage(photoPath)
        .then(result => console.log(result))
        .catch(error => console.log(error));
    }
    this.refs.toast.show("Photo path:" + photoPath, DURATION.LENGTH_SHORT);
  }

  textDetection(photoPath){
    if (Platform.OS === "android") {
      OpenCV.textDetection(photoPath)
        .then(result => {
          console.log(result);
          this.repeatPhoto();
          this.refs.toast.show("Text detected!", DURATION.LENGTH_SHORT);
        })
        .catch(error => {
          console.log(error);
          this.refs.toast.show(""+error, DURATION.LENGTH_SHORT);
        });
    } else {
      OpenCV.textDetection(photoPath)
        .then(result => console.log(result))
        .catch(error => console.log(error));
    }
    //this.refs.toast.show("Photo path:" + photoPath, DURATION.LENGTH_SHORT);
  }

  render() {
    if (this.state.photoAsBase64.isPhotoPreview) {
      return (
        <View style={styles.container}>
          <Toast ref="toast" position="center" />
          <Image
            source={{
              uri: `data:image/png;base64,${this.state.photoAsBase64.content}`
            }}
            style={styles.imagePreview}
          />
          <View style={styles.repeatPhotoContainer}>
            <TouchableOpacity onPress={this.repeatPhoto}>
              <Text style={styles.photoPreviewRepeatPhotoText}>
                Repeat photo
              </Text>
            </TouchableOpacity>
          </View>
          <View style={styles.textDetectionContainer}>
            <TouchableOpacity onPress={() => this.textDetection(this.state.photoAsBase64.photoPath)}>
              <Text style={styles.photoPreviewTextDetectionText}>
                Text detection
              </Text>
            </TouchableOpacity>
          </View>
          <View style={styles.usePhotoContainer}>
            <TouchableOpacity
              onPress={() => this.usePhoto(this.state.photoAsBase64.photoPath)}
            >
              <Text style={styles.photoPreviewUsePhotoText}>Rotate photo</Text>
            </TouchableOpacity>
          </View>
        </View>
      );
    }

    return (
      <View style={styles.container}>
        <Camera
          ref={cam => {
            this.camera = cam;
          }}
          style={styles.preview}
          permissionDialogTitle={"Permission to use camera"}
          permissionDialogMessage={
            "We need your permission to use your camera phone"
          }
        >
          <View style={styles.takePictureContainer}>
            <TouchableOpacity onPress={this.takePicture}>
              <View>
                <CircleWithinCircle />
              </View>
            </TouchableOpacity>
          </View>
        </Camera>
        <Toast ref="toast" position="center" />
      </View>
    );
  }
}

AppRegistry.registerComponent("CameraScreen", () => CameraScreen);
