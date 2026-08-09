import type { User } from "../types";

export function hasPermission(
  user: Pick<User, "permissions"> | null | undefined,
  permission: string,
): boolean {
  return Boolean(user?.permissions.includes(permission));
}
