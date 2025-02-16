import torch


def load(model: torch.nn.Module, chkpt_path: str, strict: bool = True):

    checkpoint: dict = torch.load(chkpt_path)
    weights: dict = checkpoint.get('state_dict', checkpoint)

    model_weights = model.state_dict()
    compatible_weights = {k: v for k, v in weights.items()
                          if k in model_weights}
    if strict is True:
        model.load_state_dict(compatible_weights, strict=True)
        return True

    if len(compatible_weights) != 0:
        model.load_state_dict(compatible_weights, strict=False)
        print(
            f'\033[92m   Loaded {len(compatible_weights)}/{len(model_weights)} layers\033[0m')
        return True
    else:
        return False


def save(model, optimizer: torch.optim.Optimizer, epoch: int, epoch_loss: dict, checkpoint_path: str):
    torch.save({
        'state_dict': model.state_dict(),
        'optimizer': optimizer.state_dict(),
        'epoch': epoch,
        'loss': epoch_loss,
    }, checkpoint_path)
