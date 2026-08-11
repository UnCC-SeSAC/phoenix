import cv2
import os
import argparse

def extract_frames(video_path, output_dir, interval=1):
    """
    Extract frames from a video file at a specified interval.

    Args:
        video_path (str): Path to the input video file.
        output_dir (str): Directory where the extracted frames will be saved.
        interval (int): Save one frame every `interval` frames.
    """
    if not os.path.exists(video_path):
        print(f"Error: Video file '{video_path}' not found.")
        return

    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Open the video file
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"Error: Could not open video file '{video_path}'.")
        return

    frame_count = 0
    saved_count = 0

    print(f"Starting to extract frames from '{video_path}' every {interval} frames...")

    while True:
        ret, frame = cap.read()

        # Break the loop if there are no more frames
        if not ret:
            break

        # Check if the current frame should be saved
        if frame_count % interval == 0:
            # Format the output filename (e.g., frame_0000.jpg)
            output_filename = os.path.join(output_dir, f"frame_{saved_count:04d}.jpg")
            cv2.imwrite(output_filename, frame)
            saved_count += 1
            
            if saved_count % 10 == 0:
                print(f"Saved {saved_count} frames...")

        frame_count += 1

    # Release the video capture object
    cap.release()
    print(f"Finished. Total frames read: {frame_count}, Frames saved: {saved_count}")
    print(f"Frames saved in: {output_dir}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract frames from a video file.")
    parser.add_argument("video_path", type=str, help="Path to the input video file.")
    parser.add_argument("output_dir", type=str, help="Directory to save extracted frames.")
    parser.add_argument("-n", "--interval", type=int, default=1, help="Extract a frame every N frames. Default is 1.")

    args = parser.parse_args()

    extract_frames(args.video_path, args.output_dir, args.interval)
