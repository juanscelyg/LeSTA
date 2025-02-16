"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho
File: api/dataset.py
"""


class TraversabilityDataset:

    def __init__(self, root_dir: str, phase: str, cfg: dict):
        self.dataset_type = cfg['type'].lower()
        self.dataset = self._get_dataset(root_dir, phase, cfg)

    def _get_dataset(self, dataset_root: str, phase: str, cfg: dict):
        if self.dataset_type == 'urban_traversability_dataset':
            return KITTI_Cylindrical(
                kitti_root=dataset_root,
                voxel_dim=cfg['voxel_dim'],
                phase=phase
            )
        elif self.dataset_type == '':  # add your dataset type here
            raise NotImplementedError(
                "NuScenes dataset is not implemented yet")
        else:
            raise ValueError(f"Dataset type {self.dataset_type} not supported")

    def __getitem__(self, index):
        return self.dataset[index]

    def __len__(self):
        return len(self.dataset)

    def collate_fn(self, batch):
        return self.dataset.collate_fn(batch)
