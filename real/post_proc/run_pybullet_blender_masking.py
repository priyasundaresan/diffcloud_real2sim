import os
import numpy as np
import cv2

def overlay(mask, img):
    #mask = cv2.dilate(mask,np.ones((3,4)),iterations = 7)
    mask = cv2.dilate(mask,np.ones((3,4)),iterations = 9)
    mask = np.repeat(mask[:,:,np.newaxis], 3, axis=2)
    overlay = cv2.addWeighted(img, 0.65, mask, 0.35, 0)
    return mask, overlay
    cv2.imshow('img', overlay)
    cv2.waitKey(0)

if __name__ == '__main__':
    BLENDER = '/afs/cs.stanford.edu/u/priyasun/Downloads/blender-2.82-linux64/blender'
    folders = [os.path.join('test_rollouts', f, 'output') for f in os.listdir('test_rollouts')]
    '''
    fling
    └── output
        ├── overhead
        │   ├── depth
        │   ├── mask
        │   ├── mask_dilated
        │   ├── overlay
        │   ├── overlay_dilated
        │   └── video
        └── side
            ├── depth
            ├── mask
            ├── mask_dilated
            ├── overlay
            ├── overlay_dilated
            └── video
    '''
    traj_file = 'traj_for_imgs.npz'
    cam_folders = ['overhead', 'side']

    traj_recorder_file = traj_file.replace('.npz', '_recorder.pkl')

    for f in folders:
        path_to_npz = os.path.abspath(os.path.join(f, traj_file))
        os.system('python3 pybullet_replay.py --joint_angles_file %s'%path_to_npz)
        for cam_folder in cam_folders:
            traj_dir = f
            save_dir = os.path.join(f, cam_folder)
            cam_view_mode = cam_folder

            os.system('%s -b -P render_blender_masks.py -- %s %s %s %s'%(BLENDER, traj_dir, save_dir, cam_view_mode, traj_recorder_file)) 

            mask_dir = os.path.join(save_dir, 'mask')
            out_dir = os.path.join(save_dir, 'overlay_dilated')
            out_mask_dir = os.path.join(save_dir, 'mask_dilated')
            img_dir = os.path.join(save_dir,'video')

            if not os.path.exists(out_dir):
                os.mkdir(out_dir)
            if not os.path.exists(out_mask_dir):
                os.mkdir(out_mask_dir)

            for i in range(len(os.listdir(mask_dir))):
                img = cv2.imread(os.path.join(img_dir, 'img_%d.jpg'%(i)))
                mask = cv2.imread(os.path.join(mask_dir, 'mask_%d.jpg'%i), 0)
                mask_dilated, result = overlay(mask, img)
                cv2.imwrite(os.path.join(out_dir, '%05d.jpg'%i), result)
                cv2.imwrite(os.path.join(out_mask_dir, 'mask_%d.jpg'%i), mask_dilated)
